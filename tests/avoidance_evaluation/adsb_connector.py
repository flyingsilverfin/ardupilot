from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink
import threading
import math
import time
import socket
from math import radians, cos, sin, asin, sqrt, pi

"""

Relatively basic script to receive and forward adsb_vehicle messages between SITL instances

Could be improved by adding age timeouts for connections and removing them. Currently would need to restart script to avoid sending packets to udp ports no longer being used.

"""

# prepare a mavlink connection reading on port 3000
# all SITL instances send UDP packets here
conn = mavutil.mavlink_connection("udp:127.0.0.1:3000")
# set blocking true
conn.port.setblocking(1)

EARTH_RAD = 6378137.0 #Radius of "spherical" earth

# run receiver in a separate thread so it can block
# param receive: function which receives a message and returns the message and sender address
# param handler: class with method handle_message
def threaded_reader(receive, handler):
    while True:
        msg, sender_addr = receive()
        handler.handle_message(msg, sender_addr)

# basic receiver which reads packets from conn
def receive_msg():
    msg = conn.recv_msg()
    return (msg, conn.last_address)

# for testing
def receive_filtered(type_list):
    msg = conn.recv_match(type_list)
    return (msg, conn.last_address)

#this didn't end up working
#use conn.port.sendto instead
# def udp_send_to(buf, addr):
#     # Only 1 byte arrives?
#     sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     sock.setblocking(1)
#     print sock.sendto(buf, addr)



class MessageHandler:
    def __init__(self, vehicle_manager):
        # dynamic_out and handle_config are legacy
        # only need adsb_vehicle_message now
        self.handlers_table = {
            mavlink.MAVLINK_MSG_ID_ADSB_VEHICLE : self.handle_adsb_vehicle_msg,
            mavlink.MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC: self.handle_dynamic_out_msg,
            mavlink.MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG: self.handle_config_out_msg,
        }
        self.vehicle_manager = vehicle_manager

    def handle_message(self, msg, sender_addr):
        f = self.handlers_table[msg.id]
        result = f(msg, sender_addr)

        # forward message to other SITL instances
        self.vehicle_manager.broadcast(result, sender_addr)

    def handle_adsb_vehicle_msg(self, msg, sender_addr):
        print "handling adsb_vehicle message"
        print "(%dN, %dE, %d m)" %(msg.lat, msg.lon, msg.altitude/1000)

        loc = (msg.lat, msg.lon, msg.altitude)
        self.vehicle_manager.update(loc, sender_addr)

        m = msg.get_msgbuf() #inverse of parse_char? yes, use .get_msgbuf()

        return m




class VehicleManager:
    """
    Handles vehicle ID, location/altitude, port
    Update on new location data
    forward 
    """
    def __init__(self, default_cutoff_dist = None):
        #map from addr,port to (lat,long,alt)
        self.vehicles = {}
        self.default_cutoff_dist = default_cutoff_dist

        #self.send_lock = threading.Lock()

    def update(self, loc, addr):
        self.vehicles[addr] = loc
        
    def broadcast(self, buf, source_addr):
        for addr in self.vehicles:
            if addr != source_addr and self.in_range(addr, source_addr):
                self.udp_send_to(buf, addr)
                time.sleep(0.1)
            

    #TODO test this
    def in_range(self, addr1, addr2):
        loc1 = self.vehicles[addr1]
        loc2 = self.vehicles[addr2]
        if self.default_cutoff_dist != None:
            dist = VehicleManager.dist(loc1, loc2)
            if dist < self.default_cutoff_dist:
                return True
            else:
                return False
        else:
            return True

    def udp_send_to(self, buf, addr):
        print ("Using mavlink conn port: %d to %s", (conn.port.sendto(buf, addr), str(addr)))


    @staticmethod
    def dist(loc1, loc2):
        """
        Calculate the great circle distance between two points 
        on the earth (specified in decimal degrees)
        altitude in meters!
        """
        lat1, lon1, alt1 = loc1[:3]   # trim in case heading is also appended after lat, long, alt
        lat2, lon2, alt2 = loc2[:3]
        # convert decimal degrees to radians 
        lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

        # haversine formula 
        dlon = lon2 - lon1 
        dlat = lat2 - lat1 
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * asin(sqrt(a)) 
        horizontal_dist = c * EARTH_RAD
        dist = sqrt(horizontal_dist**2 + (alt2-alt1)**2) 
        return dist




manager = VehicleManager()
thread = threading.Thread(target=threaded_reader, args=(receive_msg, MessageHandler(manager)))
thread.start()
