/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  ADSB simulator class for MAVLink ADSB peripheral
*/

#include "SIM_ADSB.h"
#include "SITL.h"

#include <stdio.h>
#include <errno.h>

#include "SIM_Aircraft.h"

namespace SITL {

SITL *_sitl;

ADSB::ADSB(const struct sitl_fdm &_fdm, const char *_home_str) :
    fdm(_fdm)
{

    float yaw_degrees;
    Aircraft::parse_home(_home_str, home, yaw_degrees);

    if (_sitl == nullptr) {
        _sitl = (SITL *)AP_Param::find_object("SIM_");
    }

    target_port = target_port_base + 10*_sitl->instance;

    receive_external_adsb_port = target_port + 1;

    
    

    //bool success = receive_external_adsb.bind(target_address, receive_external_adsb_port);
    //::printf("Bound tcp receive to port %d with success: ", receive_external_adsb_port);
    //bool success = adsb_coordinator.bind("127.0.0.1", receive_external_adsb_port);
    if (!adsb_coordinator.bind("0.0.0.0", receive_external_adsb_port)) {
        ::fprintf(stderr, "SITL: socket in bind failed on sim in : %d  - %s\n", receive_external_adsb_port, strerror(errno));
        ::fprintf(stderr, "Abording launch...\n");
        exit(1);
    }
    
    //::printf("Bound receive to port %d with success: ", receive_external_adsb_port);

    adsb_coordinator.connect(target_address, coordinator_port);
    ::printf("Connect default to port %d", coordinator_port);


    adsb_coordinator.reuseaddress();
    adsb_coordinator.set_blocking(false);
    
    // preset what we can
    this_adsb_vehicle.flags = ADSB_FLAGS_VALID_COORDS |
                ADSB_FLAGS_VALID_ALTITUDE |
                ADSB_FLAGS_VALID_HEADING |
                ADSB_FLAGS_VALID_VELOCITY |
                ADSB_FLAGS_VALID_CALLSIGN;
    this_adsb_vehicle.altitude_type = ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
    this_adsb_vehicle.emitter_type = ADSB_EMITTER_TYPE_UAV;
    this_adsb_vehicle.squawk = 0; // NOTE: ADSB_FLAGS_VALID_SQUAWK bit is not set


    
}


/*
  update a simulated vehicle
 */
void ADSB_Vehicle::update(float delta_t)
{
    if (!initialised) {
        initialised = true;
        ICAO_address = (uint32_t)(rand() % 10000);
        snprintf(callsign, sizeof(callsign), "SIM%u", ICAO_address);
        position.x = Aircraft::rand_normal(0, _sitl->adsb_radius_m);
        position.y = Aircraft::rand_normal(0, _sitl->adsb_radius_m);
        position.z = -fabsf(_sitl->adsb_altitude_m);

        double vel_min = 5, vel_max = 20;
        if (position.length() > 500) {
            vel_min *= 3;
            vel_max *= 3;
        } else if (position.length() > 10000) {
            vel_min *= 10;
            vel_max *= 10;
        }
        velocity_ef.x = Aircraft::rand_normal(vel_min, vel_max);
        velocity_ef.y = Aircraft::rand_normal(vel_min, vel_max);
        velocity_ef.z = Aircraft::rand_normal(0, 3);
    }

    position += velocity_ef * delta_t;
    if (position.z > 0) {
        // it has crashed! reset
        initialised = false;
    }
}

/*
  update the ADSB peripheral state
*/
void ADSB::update(void)
{

    if (_sitl == nullptr) {
        _sitl = (SITL *)AP_Param::find_object("SIM_");
        return;
    } else if (_sitl->adsb_plane_count <= 0) {
        //return;
        num_vehicles = 0;  // override to get to send_report no matter what, just skip the vehicle updating
    } else if (_sitl->adsb_plane_count >= num_vehicles_MAX) {
        _sitl->adsb_plane_count.set_and_save(0);
        num_vehicles = 0;
        return;
    } else if (num_vehicles != _sitl->adsb_plane_count) {
        num_vehicles = _sitl->adsb_plane_count;
        for (uint8_t i=0; i<num_vehicles_MAX; i++) {
            vehicles[i].initialised = false;
        }
    }

    // calculate delta time in seconds
    uint32_t now_us = AP_HAL::micros();

    float delta_t = (now_us - last_update_us) * 1.0e-6f;
    last_update_us = now_us;

    for (uint8_t i=0; i<num_vehicles; i++) {
        vehicles[i].update(delta_t);
    }

    //
    
    // see if we should do a report
    send_report();
    receive_external_coordinator_messages();
}

/*
  send a report to the vehicle control code over MAVLink
*/
void ADSB::send_report(void)
{
    if (AP_HAL::millis() < 10000) {
        // simulated aircraft don't appear until 10s after startup. This avoids a windows
        // threading issue with non-blocking sockets and the initial wait on uartA
        return;
    }
    if (!mavlink.connected && mav_socket.connect(target_address, target_port)) {
        ::printf("ADSB connected to %s:%u\n", target_address, (unsigned)target_port);
        mavlink.connected = true;
    }
    if (!mavlink.connected) {
        return;
    }

    // check for incoming MAVLink messages
    uint8_t buf[100];
    ssize_t ret;


    while ((ret=mav_socket.recv(buf, sizeof(buf), 0)) > 0) {
        for (uint8_t i=0; i<ret; i++) {
            mavlink_message_t msg;
            mavlink_status_t status;
            if (mavlink_frame_char_buffer(&mavlink.rxmsg, &mavlink.status,
                                          buf[i],
                                          &msg, &status) == MAVLINK_FRAMING_OK) {
                switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    if (!seen_heartbeat) {
                        seen_heartbeat = true;
                        vehicle_component_id = msg.compid;
                        vehicle_system_id = msg.sysid;
                        //::printf("ADSB using srcSystem %u from port %u\n", (unsigned)vehicle_system_id, (unsigned) target_port);
                    }
                    break;
                }

                case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC: {
                    //::printf("Received MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC message from drone controller");
                    // this already has a transmit rate set in the firmware, don't need to check output rate here
                    // just parse and generate ADSB_VEHICLE msg pass on to coordinator

                    handle_adsb_dynamic_out_message(msg);
                    transmit_adsb_vehicle_msg();
                    break;
                }
                case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG: {
                    handle_adsb_cfg_message(msg);
                    break;
                }    
                default: {
                    //::printf("Received copter message in SIM_ADSB with id: %d \n", msg.msgid);
                    
                    break;
                }
                }
            }
        }
    }

    if (!seen_heartbeat) {
        return;
    }

    mavlink_message_t msg;
    uint16_t len;
    uint32_t now = AP_HAL::millis();

    if (now - last_heartbeat_ms >= 1000) {
        mavlink_heartbeat_t heartbeat;
        heartbeat.type = MAV_TYPE_ADSB;
        heartbeat.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
        heartbeat.base_mode = 0;
        heartbeat.system_status = 0;
        heartbeat.mavlink_version = 0;
        heartbeat.custom_mode = 0;

        /*
          save and restore sequence number for chan0, as it is used by
          generated encode functions
         */
        mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
        uint8_t saved_seq = chan0_status->current_tx_seq;
        chan0_status->current_tx_seq = mavlink.seq;
        len = mavlink_msg_heartbeat_encode(vehicle_system_id,
                                           vehicle_component_id,
                                           &msg, &heartbeat);
        chan0_status->current_tx_seq = saved_seq;

        mav_socket.send(&msg.magic, len);

        last_heartbeat_ms = now;
    }


    /*
      send a ADSB_VEHICLE messages
     */
    uint32_t now_us = AP_HAL::micros();
    if (now_us - last_report_us >= reporting_period_ms*1000UL) {
        for (uint8_t i=0; i<num_vehicles; i++) {
            ADSB_Vehicle &vehicle = vehicles[i];
            Location loc = home;

            location_offset(loc, vehicle.position.x, vehicle.position.y);

            // re-init when exceeding radius range
            if (get_distance(home, loc) > _sitl->adsb_radius_m) {
                vehicle.initialised = false;
            }
            
            mavlink_adsb_vehicle_t adsb_vehicle {};
            last_report_us = now_us;

            adsb_vehicle.ICAO_address = vehicle.ICAO_address;
            adsb_vehicle.lat = loc.lat;
            adsb_vehicle.lon = loc.lng;
            adsb_vehicle.altitude_type = ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            adsb_vehicle.altitude = -vehicle.position.z * 1000;
            adsb_vehicle.heading = wrap_360_cd(100*degrees(atan2f(vehicle.velocity_ef.y, vehicle.velocity_ef.x)));
            adsb_vehicle.hor_velocity = norm(vehicle.velocity_ef.x, vehicle.velocity_ef.y) * 100;
            adsb_vehicle.ver_velocity = -vehicle.velocity_ef.z * 100;
            memcpy(adsb_vehicle.callsign, vehicle.callsign, sizeof(adsb_vehicle.callsign));
            adsb_vehicle.emitter_type = ADSB_EMITTER_TYPE_LARGE;
            adsb_vehicle.tslc = 1;
            adsb_vehicle.flags =
                ADSB_FLAGS_VALID_COORDS |
                ADSB_FLAGS_VALID_ALTITUDE |
                ADSB_FLAGS_VALID_HEADING |
                ADSB_FLAGS_VALID_VELOCITY |
                ADSB_FLAGS_VALID_CALLSIGN |
                ADSB_FLAGS_SIMULATED;
            adsb_vehicle.squawk = 0; // NOTE: ADSB_FLAGS_VALID_SQUAWK bit is not set

            mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
            uint8_t saved_seq = chan0_status->current_tx_seq;
            chan0_status->current_tx_seq = mavlink.seq;
            len = mavlink_msg_adsb_vehicle_encode(vehicle_system_id,
                                                  MAV_COMP_ID_ADSB,
                                                  &msg, &adsb_vehicle);
            chan0_status->current_tx_seq = saved_seq;
            
            uint8_t msgbuf[len];
            len = mavlink_msg_to_send_buffer(msgbuf, &msg);
            if (len > 0) {
                mav_socket.send(msgbuf, len);
            }
        }
    }
    
    // ADSB_transceiever is enabled, send the status report.
    if (_sitl->adsb_tx && now - last_tx_report_ms > 1000) {
        last_tx_report_ms = now;

        mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);    // this goes to GCS
        uint8_t saved_seq = chan0_status->current_tx_seq;
        uint8_t saved_flags = chan0_status->flags;
        chan0_status->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
        chan0_status->current_tx_seq = mavlink.seq;
        const mavlink_uavionix_adsb_transceiver_health_report_t health_report = {UAVIONIX_ADSB_RF_HEALTH_OK};
        len = mavlink_msg_uavionix_adsb_transceiver_health_report_encode(vehicle_system_id,
                                              MAV_COMP_ID_ADSB,
                                              &msg, &health_report);
        chan0_status->current_tx_seq = saved_seq;
        chan0_status->flags = saved_flags;

        uint8_t msgbuf[len];
        len = mavlink_msg_to_send_buffer(msgbuf, &msg);
        if (len > 0) {
            mav_socket.send(msgbuf, len);
            ::printf("ADSBsim send tx health packet\n");
        }
    }
}

void ADSB::handle_adsb_cfg_message(mavlink_message_t &msg) {
    /*
        I'm just going to assume the ADSB transmitter is close enough to center of aircraft
        Need to save: ICAO, callsign only
        contents: ['ICAO', 'stallSpeed', 'callsign', 'emitterType', 'aircraftSize', 'gpsOffsetLat', 'gpsOffsetLon', 'rfSelect']
    */

    uint32_t icao = mavlink_msg_uavionix_adsb_out_cfg_get_ICAO(&msg);
    char callsign[9]; 
    mavlink_msg_uavionix_adsb_out_cfg_get_callsign(&msg, callsign);

    this_adsb_vehicle.ICAO_address = icao;
    if (strcmp(callsign, this_adsb_vehicle.callsign) != 0) { // only do a write if not the same (unlikely to change)
        memcpy(this_adsb_vehicle.callsign, callsign, sizeof(this_adsb_vehicle.callsign));
    }

}

void ADSB::handle_adsb_dynamic_out_message(mavlink_message_t msg) {
//['utcTime', 'gpsLat', 'gpsLon', 'gpsAlt', 'baroAltMSL', 'accuracyHor', 'accuracyVert', 'accuracyVel', 'velVert', 'velNS', 'VelEW', 'state', 'squawk', 'gpsFix', 'numSats', 'emergencyStatus']

    mavlink_uavionix_adsb_out_dynamic_t dynamic_out {};
    mavlink_msg_uavionix_adsb_out_dynamic_decode(&msg, &dynamic_out);


    this_adsb_vehicle.lat = dynamic_out.gpsLat;
    this_adsb_vehicle.lon = dynamic_out.gpsLon;
    this_adsb_vehicle.altitude = dynamic_out.gpsAlt;
    this_adsb_vehicle.heading = wrap_360_cd(100*degrees(atan2f(dynamic_out.velNS, dynamic_out.VelEW)));
    //NOTE: it is .VelEW with capital V - probably just a typo? In python too though
    this_adsb_vehicle.hor_velocity = norm(dynamic_out.VelEW, dynamic_out.velNS); //already in cm/s
    this_adsb_vehicle.ver_velocity = dynamic_out.velVert;    // already in cm/2

}

/*
    Uses stored state in this instance to transmit data as an ADSB_VEHICLE message
    State is mostly stored in this_adsb_vehicle;
*/
void ADSB::transmit_adsb_vehicle_msg() {

    uint32_t now = AP_HAL::millis();
    uint32_t delta = now - last_this_vehicle_adsb_out;
    delta = delta/1000 == 0? 1 : delta/1000; // give "1" second interval if <1000 millis
    last_this_vehicle_adsb_out = now;

    this_adsb_vehicle.tslc = (uint8_t) delta;    // set last parameter

    mavlink_message_t msg;
    uint16_t len = mavlink_msg_adsb_vehicle_encode(vehicle_system_id,
                                            MAV_COMP_ID_ADSB,
                                            &msg, &this_adsb_vehicle);
    uint8_t msgbuf[len];
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);
    if (len > 0) {
        adsb_coordinator.send(msgbuf, len);
    }
}

void ADSB::receive_external_coordinator_messages() {
    /*
        Addition to check for reports from coordinator
    */


    uint8_t buf[100];
    ssize_t ret;


    errno = 0;

    do  {
        ret = adsb_coordinator.recv(ptr, sizeof(buf), 0);
        if (ret < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != 0) {
                fprintf(stderr, "error recv on socket in: %s \n",
                        strerror(errno));
            }
            break;
        }

        ::printf("\n received something: %d bytes", (int32_t)ret);

        for (uint8_t i=0; i<num_received; i++) {
            mavlink_message_t msg;
            mavlink_status_t status;
            if (mavlink_frame_char_buffer(&mavlink_external.rxmsg, &mavlink_external.status,
                                        buf[i],
                                        &msg, &status) == MAVLINK_FRAMING_OK) {
                switch (msg.msgid) {
                    case MAVLINK_MSG_ID_ADSB_VEHICLE: {
                        //mavlink_adsb_vehicle_t vehicle;
                        //mavlink_msg_adsb_vehicle_decode(&msg, &vehicle);
                        ::printf("\nReceived mavlink adsb vehicle message");

                        handle_external_coordinator_message(msg);

                        break;
                    }
                    default : {
                        ::printf("\nReceived non-ADSB_VEHICLE message on external link: %u", msg.msgid);
                        break;
                    }
                }
            }
        }
    } while ( ret > 0 );
}

void ADSB::handle_external_coordinator_message(mavlink_message_t &msg) {
    if (!mavlink.connected && mav_socket.connect(target_address, target_port)) {
        ::printf("ADSB connected to %s:%u\n", target_address, (unsigned)target_port);
        mavlink.connected = true;
    }
    if (!mavlink.connected) {
        return;
    }

    uint8_t msgbuf[500];
    uint16_t len = mavlink_msg_to_send_buffer(msgbuf, &msg);
    if (len > 0) {
        ssize_t sent = mav_socket.send(msgbuf, len);
        ::printf("\nManaged to send %d byte", sent);
    }

}


} // namespace SITL
