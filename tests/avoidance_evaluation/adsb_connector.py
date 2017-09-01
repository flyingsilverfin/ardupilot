from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink
import threading
import math
import time
import socket
from math import radians, cos, sin, asin, sqrt, pi


conn = mavutil.mavlink_connection("udp:127.0.0.1:3000")
conn.port.setblocking(1)


mv = mavlink.MAVLink(conn)

def threaded_reader(receive, handler):
    while True:
        msg, sender_addr = receive()
        handler.handle_message(msg, sender_addr)

def receive_msg():
    msg = conn.recv_msg()
    return (msg, conn.last_address)

def receive_filtered(type_list):
    msg = conn.recv_match(type_list)
    return (msg, conn.last_address)
"""    
def receive_all():
    buf = conn.recv()
    return (buf, conn.last_address)
    
def parse_raw(buf):
    msg = mv.parse_char(buf)
    print msg
    return msg

"""
def udp_send_to(buf, addr):
    # Only 1 byte arrives?
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(1)
    print sock.sendto(buf, addr)




class MessageHandler:
    def __init__(self, vehicle_manager):
        self.handlers_table = {
            mavlink.MAVLINK_MSG_ID_ADSB_VEHICLE : self.handle_adsb_vehicle_msg,
            mavlink.MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC: self.handle_dynamic_out_msg,
            mavlink.MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG: self.handle_config_out_msg,
        }
        self.vehicle_manager = vehicle_manager

    def handle_message(self, msg, sender_addr):
        f = self.handlers_table[msg.id]
        result = f(msg, sender_addr)

        self.vehicle_manager.broadcast(result, sender_addr)

    def get_squawk(self, msg):
        return msg.squawk     #hack until sort out proper retrieval mechanism for names

    def get_id(self, msg):
        return self.get_squawk(msg)

    def handle_adsb_vehicle_msg(self, msg, sender_addr):
        print "handling adsb_vehicle message"
        print "(%dN, %dE, %d m)" %(msg.lat, msg.lon, msg.altitude/1000)

        loc = (msg.lat, msg.lon, msg.altitude)
        self.vehicle_manager.update(loc, sender_addr)

        #TODO: turn msg into byte buffer
        testing = msg.get_msgbuf() #inverse of parse_char?

        return testing

    def handle_dynamic_out_msg(self, msg):
        #TODO translate into ADSB_VEHICLE, return that message
        print "handling dynamic_out message - this shouldn't be arriving anymore"
        return None

    def handle_config_out_msg(self, msg):
        #ignore for now
        print "handling config_out message - this shouldn't be arriving anymore"
        return None



class VehicleManager:
    """
    Handles vehicle ID, location/altitude, port
    Update on new location data
    forward 

    """
    def __init__(self, default_cutoff_dist = None):
        #map from port to (lat,long,alt)
        self.vehicles = {}
        self.default_cutoff_dist = default_cutoff_dist

        self.out_port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.out_port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.out_port.bind(('127.0.0.1', 2999))
        self.out_port.setblocking(1)

        self.send_lock = threading.Lock()

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
        #self.send_lock.acquire()
        #print("Send %d bytes to %s" %(self.out_port.sendto(buf, addr), str(addr)))
        #self.send_lock.release()
        
        #print("Send %d bytes to %s" %(self.out_port.sendto(buf, addr), str(addr)))

        print ("Using mavlink conn port: %d to %s", (conn.port.sendto(buf, addr), str(addr)))


        #sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #sock.setblocking(1)
        #print sock.sendto(buf, addr)
        #udp_send_to(buf, addr)

    @staticmethod
    def dist(loc1, loc2):
        """
        This method is an approximation, and will not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        dlat = loc2[0] - loc1[0]
        dlong = loc2[1] - loc1[1]
        dist = math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
        return dist




manager = VehicleManager()
thread = threading.Thread(target=threaded_reader, args=(receive_msg, MessageHandler(manager)))
thread.start()
