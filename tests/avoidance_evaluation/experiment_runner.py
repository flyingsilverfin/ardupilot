import sys
from dronekit import connect, VehicleMode, LocationGlobal
from pymavlink import mavutil
import json
import os
import shutil
import subprocess
from multiprocessing.dummy import Pool as ThreadPool
import time
import threading
import math

CONNECTION_IP = '127.0.0.1'
DISTANCE_TOLERANCE = 0.5 # must be this far from a point to be considered "there"
FNULL = open(os.devnull, 'w')   # equivalent to > /dev/null


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

class VehicleCommander():
    def __init__(self, id, vehicle, home, flight_plan, forwarder):
        self.vehicle = vehicle
        self.home = home
        self.plan = flight_plan
        self.id = id
        self.forwarder = forwarder
        
        self.vehicle.parameters['SYSID_THISMAV'] = id

        self.vehicle.add_message_listener('*', self.msg_listener)


    def msg_listener(self, vehicle, name, message):
        #if message.get_srcSystem() != self.id:
            #print "mismatch: %d vs %d, fresh from params is: %d" %(message.get_srcSystem(), self.id, vehicle.parameters['SYSID_THISMAV'])
            #stupid hack to set src system correctly
            #message.get_header().srcSystem = self.id
        self.forwarder.forward(message)
            

    def run(self):
        print "connect now!"
        time.sleep(5)
        v = self.vehicle
        while not v.is_armable:
            time.sleep(0.5)
        v.mode = VehicleMode('GUIDED')
        v.armed = True
        while not v.armed:
            time.sleep(0.1)

        height = self.plan['0']['waypoint'][2] - self.home[2]
        speed = self.plan['0']['speed_to']
        self.takeoff(height, speed)
        #v.simple_takeoff(alt=self.plan['0']['waypoint'][2])
        #time.sleep(10)

        next = 0
        #for pt in self.plan:
        if len(self.plan) > 1:
            while True:
                print "going to: " + str(self.plan[str(next)])
                dest = self.plan[str(next)]['waypoint']
                speed_to = self.plan[str(next)]['speed_to']
                self.goto(dest, speed_to)
                next = (next+1)%(len(self.plan))
        else:
            dest = self.plan["0"]['waypoint']
            speed_to = self.plan["0"]['speed_to']
            self.goto(dest, speed_to)
            #dest = LocationGlobal(*self.plan[str(next)]['waypoint'])
            #v.simple_goto(dest)

    def takeoff(self, height, speed):
        self.vehicle.airspeed = speed
        self.vehicle.simple_takeoff(alt=height)
        while True:
            if self.vehicle.location.global_relative_frame.alt>=height*0.95:
                print "Reached target altitude"
                break
            time.sleep(0.1)

    def goto(self, dest, speed):
        self.vehicle.simple_goto(LocationGlobal(*dest), airspeed=speed)
        while dist(self.current_location(), dest) > DISTANCE_TOLERANCE:
            time.sleep(0.1)

    def current_location(self):
        loc = self.vehicle.location.global_frame
        return (loc.lat, loc.lon, loc.alt)


class VehicleRunner():
    def __init__(self, experiment_name, id, specification, forwarder):
        self.id = id
        self.exp_name = experiment_name

        #self.sitl = specification['sitl_port'] # deprecated
        self.master = specification['master']
        #self.name = specification['name']
        #self.out_ports = specification['extra_out_ports'] #deprecated
        #self.out_ports.append(3001) #TESTING port for everyone to forward to MP with UDP DOESN'T WORK
        self.home = specification['home']
        self.instance = specification['instance']
        self.flight_plan = specification['flight_plan']

        #self.mavproxy = None
        self.sitl_instance = None
        self.vehicle = None
        self.vehicle_commander = None

        self.forwarder = forwarder

        self.init()

    def init(self):
        print("Initializing vehicle %d" %self.instance)
        self.start_sitl()
        #self.start_mavproxy()

    """
    #Not actually needed!

        def start_mavproxy(self):
            #mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --aircraft Test --out udp:127.0.0.1:5502
            cmd = ["mavproxy.py"]
            cmd.extend(["--master", format("tcp:%s:%d" %(CONNECTION_IP, self.master))])
            cmd.extend(["--sitl", format("%s:%d" %(CONNECTION_IP, self.sitl))])
            cmd.extend(["--aircraft", self.name])
            for port in self.out_ports:
                cmd.extend(["--out", format("udp:%s:%d" %(CONNECTION_IP, port))])
            #print(cmd)
            self.mavproxy = subprocess.Popen(cmd, stdout=FNULL, stderr=subprocess.STDOUT, close_fds=True)
    """
    def start_dronekit(self):
        #connect directly to SITL
        self.vehicle = connect(format("tcp:%s:%d" %(CONNECTION_IP, self.master)), wait_ready=True)
        self.vehicle_commander = VehicleCommander(self.id, self.vehicle, self.home, self.flight_plan, self.forwarder)


    def start_sitl(self):
        
        log_folder = os.path.join('.', 'logs', self.exp_name, 'vehicle_' + str(self.id))
        os.mkdir(log_folder)
        

        #~/dev/ardupilot/build/sitl-debug/bin/arducopter -S -I0 --home -35.663261,149.165230,584,353 --model + --speedup 1 --defaults /home/joshua/dev/evaluation/copter_params.parm --wipe
        cmd = ["../../build/sitl-debug/bin/arducopter"]
        cmd.append('-S')
        cmd.append('--wipe')
        cmd.extend(['--model', '+'])
        cmd.extend(['--speedup', '2'])
        cmd.append(format("-I%d" %(self.instance)))
          
        h = self.home
        home_str = format("%s,%s,%s,%s" %(h[0], h[1], h[2], h[3]))
        cmd.extend(["--home", home_str])

        cmd.extend(["--defaults", format("/home/joshua/dev/evaluation/copter_parameters/copter_params_%d.parm" %(self.id+1))])
        #print(cmd)
        logfile = open('/tmp/logs/log_sitl_' + str(self.id), 'w')
        self.sitl_instance = subprocess.Popen(cmd, 
                                                stdout=logfile, 
                                                stderr=subprocess.STDOUT, 
                                                close_fds=True,
                                                cwd=log_folder)

    def run(self):
        self.start_dronekit()
        if self.vehicle is None:
            print "Dronekit vehicle is not instantiated yet, aborting"
            return
        self.vehicle.parameters['SYSID_THISMAV'] = self.id
        self.vehicle_commander.run()

    def exit(self):
        #self.mavproxy.kill()
        self.sitl_instance.kill()
        self.vehicle.close()

    


# used to forward to a GCS for visualization
# combines different mavlink streams into one
class MavlinkJoiner():
    def __init__(self, destination_addresses):
        self.forward_to = destination_addresses
        self.conns = []
        self.lock = threading.Lock()
        for addr in self.forward_to:
            try:
                self.conns.append(mavutil.mavlink_connection(addr))
            except Exception as e:
                print("failed to set up mavlink connection to %s" %(addr))
        
    def close_all(self):
        for conn in self.forward_to:
            conn.close()

    def forward(self, msg):
        #don't want another pool thread writing any bytes at the same time
        self.lock.acquire()
        for c in self.conns:
            c.write(msg.get_msgbuf())
        self.lock.release()
    

class ExperimentRunner():
    def __init__(self, experiment_file, run_when_ready=True):
        split_path = experiment_file.split('/')
        self.experiment_name = split_path[-2]

        logging_folder = os.path.join('.', 'logs', self.experiment_name)
        try:
            os.mkdir(logging_folder)
        except OSError:
            shutil.rmtree(logging_folder, ignore_errors=True)
            os.mkdir(logging_folder)

        with open(experiment_file) as plan_file:
            self.plan = json.load(plan_file)
        self.vehicles = []
        self.pool = ThreadPool(len(self.plan['vehicles'])) 
        self.forwarder = MavlinkJoiner(["udpout:127.0.0.1:3002"]) #hook up ground station for multi-vehicle visualisation
        if run_when_ready:
            self.run()

    def run(self):
        print "Executing all vehicle plans"
        for key in self.plan['vehicles']:
            num = int(key)
            self.vehicles.append(VehicleRunner(self.experiment_name, num, self.plan['vehicles'][key], self.forwarder))

        self.pool.map(self._exec, self.vehicles)

    def _exec(self, vehicle):
        vehicle.run()

    def exit(self):
        for v in self.vehicles:
            v.exit()


if __name__ == '__main__':
    args = sys.argv
    if len(args) <= 1:
        print "Need to supply the filename of the mission. Aborting."
    else:
        filename = args[1]
        runner = ExperimentRunner(filename, run_when_ready=False)
        waiting = raw_input("Press enter to begin!")
        runner.run()
        waiting = raw_input()
        runner.exit()

