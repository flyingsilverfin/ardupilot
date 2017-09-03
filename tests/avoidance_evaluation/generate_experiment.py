"""
Basic Experiment features:
    N Vehicles, each with
     Home position
     List of waypoints in GPS (lat, lng, altitude) with speed to get there
     instance number
     relevant ports (for mavproxy, dronekit)
"""

import ast
import random
import numpy as np
from math import radians, cos, sin, asin, sqrt, pi, atan2
import json
import glob
import os
import sys
from operator import add
import argparse

#this is the default aircraft home when spawned
DEFAULT_HOME = (-35.663261, 149.165230, 584, 0)   #(lat, lng, altitude (meters), heading - vs North)
DEFAULT_TAKEOFF = 30        #default height to takeoff to do maneuvers

ANY_LENGTH = -1     # constant signal value for any length lists in parser
USE_ALL_DEFAULTS = False
EARTH_RAD = 6378137.0 #Radius of "spherical" earth

instance = None

# ----- helper functions -----

def rad_to_deg(rad):
    return (rad/pi) * 180

def deg_to_rad(deg):
    return deg/180 * pi

def meters_to_lat(meters):
    dLat = meters/EARTH_RAD
    return rad_to_deg(dLat)

def meters_to_lng(original_location, meters):
    dLng = meters/(EARTH_RAD*cos(pi*original_location[0]/180))
    return rad_to_deg(dLng)

# get a normally distributed number with params mean and stddev
def get_normal(mean, stddev):
    if stddev == 0:
        return mean
    else:
        return np.random.normal(mean, stddev, 1)

def random_perturb_horizontal(pos, meters):
    perturbation = [random.uniform(0, meters_to_lat(meters)), random.uniform(0, meters_to_lng(pos, meters))]
    extended = perturbation + [0]*(len(pos) - 2)
    return map(add, pos, extended)


# given two (lat, lon, altitude) pairs calculate their distance in meters plus the angle from p1 to p2
def get_distance_angle(p1, p2):
    """
    Calculate the great circle distance between two points 
    on the earth (specified in decimal degrees)
    altitude in meters!
    """
    lat1, lon1, alt1 = p1[:3]   # trim in case heading is also appended after lat, long, alt
    lat2, lon2, alt2 = p2[:3]
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    horizontal_dist = c * EARTH_RAD
    dist = sqrt(horizontal_dist**2 + (alt2-alt1)**2)    # straight line approx works well enough over short distances
    
    
    dlat = lat2 - lat1
    dlong = lon2 - lon1
    dx = cos(pi/180*lat1)*(dlong)    # for angle  
    angle = atan2(dx, dlat)
    return (dist, angle)

# given a GPS coordinate, find the coordinate a certain number of meters in the East/West and North/South direction
def get_location_metres(original_location, dNorth, dEast):
    """
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    #Coordinate offsets in radians
    dLat = dNorth/EARTH_RAD
    dLon = dEast/(EARTH_RAD*cos(pi*original_location[0]/180))

    #New position in decimal degrees
    newlat = original_location[0] + (dLat * 180/pi)
    newlon = original_location[1] + (dLon * 180/pi)
    
    return (newlat, newlon)

#0 degrees is true north, 180deg is south
# similar to above, but provides a  GPS coordinate a distance in a certain direction
def gps_from(start, distance, angle_deg):
    angle_rad = deg_to_rad(float(angle_deg))
    dy = distance*cos(angle_rad)
    dx = distance*sin(angle_rad)
    print dy, dx, angle_rad
    return get_location_metres(start, dy, dx)


# ----- classes for generating a certain kind of collision -----


# pretty useless container for flying straight line segments
class StraightLine():
    def __init__(self, id, home):
        self.id = id
        self.home = home
        self.waypoints = []

    def line_to(self, pos, speed=5):
        self.waypoints.append({"waypoint":pos, "speed_to": speed})

    # def get_points(self):
    #     pts = {}
    #     for i in range(len(self.waypoints)):
    #         pts[i] = self.waypoints[i]
    #     return pts

    def get_waypt(self, n):
        return self.waypoints[n]

    def get_home(self):
        return self.home

# Could imagine other segments with arcs etc. but these are actually pretty hard to do

class Hovering():
    def __init__(self, id, home, height, speed=5):
        self.id = id
        self.home = home
        self.height = height
        self.speed = speed
        pt_0 = map(add, self.home[:3], (0,0,height))
        print pt_0
        self.waypoints = [{
                "waypoint": pt_0,
                "speed_to": self.speed
            }]

    def get_waypt(self, n):
        return self.waypoints[n]

    def get_target_height(self):
        return self.waypoints[0]['waypoint'][2]

    def get_home(self):
        return self.home


#generate N vehicles, of which vehicle 0 is hovering and the rest are position on a circle around
# their distances are controlled by a random variable with average distance and variance given. 
# 0 variance = equidistant
class Hover_LineInto():
    def __init__(self, n_line=1, av_dist=30, dist_variance=0):
        n_line = int(n_line)
        av_dist = float(av_dist)
        dist_variance = float(dist_variance)
        self.n_line = n_line
        self.av_dist = av_dist
        self.dist_variance = dist_variance

        #add a random pertubation to default home so not always in exact same place
        self.home = random_perturb_horizontal(DEFAULT_HOME, 20)
        self.vehicles = {}
        self.vehicles[0] = Hovering(0, self.home, DEFAULT_TAKEOFF)

        #print self.vehicles[0].home, self.vehicles[0].waypoints

        self.target_height = self.vehicles[0].get_target_height()

        for i in range(1, 1 + n_line):
            # get GPS coordinates of start position on the circle for this vehicle
            v_home = self.start_pos_latlng(i)
            v_home += (self.home[2], 0) #append height, heading
            # initialize the vehicle
            v = StraightLine(i, v_home)
            (dist, angle) = get_distance_angle(self.home, v.home)
            #print dist, angle, "\n"
            # add a new segment going through hovering position to the opposite side of the circle
            (new_lat, new_lng) = gps_from(v.home, dist*2, 180+rad_to_deg(angle))
            v.line_to((new_lat, new_lng, self.vehicles[0].get_target_height()))
            self.vehicles[i] = v
    
    #start all line_into vehicles in a n-gon around hoverer
    def start_pos_latlng(self, n):
        angle = 360.0 * n / self.n_line
        dist = get_normal(self.av_dist, self.dist_variance**0.5)
        return gps_from(self.home[:3], dist, angle)

    # def get_plans(self):
    #     vehicle_plans = {}
    #     for i in self.vehicles:
    #         v = self.vehicles[i]
    #         vehicle_plans[i] = {
    #             "flight_plan": v.get_points()
    #         }
    #     return vehicle_plans

    def get_waypoint(self, vehicle, waypoint_num):
        return self.vehicles[vehicle].get_waypt(waypoint_num)
    
    def get_num_waypts(self, vehicle):
        return len(self.vehicles[vehicle].waypoints)

    def num_vehicles(self):
        return len(self.vehicles)

    def get_home(self, vehicle):
        print "getting home for vehicle %d" %vehicle
        return self.vehicles[vehicle].get_home()

#   --------------------------------------------------------
#   --------------------------------------------------------
#   --------------------------------------------------------


"""

    The dictionaries below are what I think is a pretty neat way of generating a JSON settings file
    They essentially form a tree of settings, options etc. that need to be set
    They are parsed further down. The last Dictionary is a the root

    This initially was meant for an interactive terminal menu but evolved into automation-capable system

    To enable this the 'default' option can be a lambda which takes arguments
        arguments are specified in 'default_args'. Similar in format to 'suboption_args', these can be of two forms for now:
            'arg:x' - this tells the parser to take the argument x passed in from the parent node
            'this:y' - tells the parser to retrieve the value y set in a sibling prompt (ie. in the same)

    Similar to 'default', we have 'suboption', which means this option needs to be recursively explored
        When getting information in the suboption, there may be questions which have defaults with lambdas which need arguments
        These arguments are passed in, as described as above
        An enumerator _n is passed in as well if it's a repeated suboption (for example for waypoints)

    The last important aspect is 'return'
        This is specified for options which ask for suboptions to be explored
        currently I'm using 'dict_to_list' and 'use_enumerated_dict'
            'dict_to_list' requires that 'return_params' also be set: this contains an order in which keys are extracted into the list
    
    The comments annotating the options should help understand what's going on I hope
    A typed language would've made debugging this much easier :/
"""


# options for home lat/lng/altitude/heading
home_options = {
    '_order': ['lat', 'lng', 'alt', 'heading'],         # order to request the data in this options dictionary
    'lat': {
        'prompt': 'Home Latitude',
        'type': float,
        'default': lambda: DEFAULT_HOME[0]+ random.uniform(0.0001, 0.001)
    },
    'lng': {
        'prompt': 'Home Longitude',
        'type': float,
        'default': lambda: DEFAULT_HOME[1]+ random.uniform(0.0001, 0.001)
    },
    'alt': {
        'prompt': 'Home Altitude',
        'type': float,
        'default': DEFAULT_HOME[2]
    },
    'heading': {
        'prompt': 'Home Heading (degrees CW from North)',
        'type': float,
        'default': DEFAULT_HOME[3]
    }
}

# options for setting any waypoint
# defaults take three arguments: the home position set, the ID of this vehicle, and which waypoint is currently being set
position_options = {
    '_order': ['lat', 'lng', 'alt'],
    'lat': {
        'prompt': 'Latitude',
        'type': float,
        'default': lambda home, id, n: home[0] + random.uniform(0.0001, 0.001),
        'default_args': 'arg:home, arg:id, arg:num_waypoint'
    },
    'lng': {
        'prompt': 'Longitude',
        'type': float,
        'default': lambda home, id, n: home[1] + random.uniform(0.0001, 0.001),
        'default_args': 'arg:home, arg:id, arg:num_waypoint'
    },
    'alt': {
        'prompt': 'Altitude',
        'type': float,
        'default': lambda home, id, n: home[2] + DEFAULT_TAKEOFF,
        'default_args': 'arg:home, arg:id, arg:num_waypoint'
    }
}



#   To make the original setup work with the class-based approach to creating flight plans with specific conflicts
#   These functions are called by the defaults lambdas
#   They use the global 'instance' of say Hover_LineInto and get the required data from the instance

# this function, given a home location, id either generates the next waypoint randomly, or gets the next waypoint from 'instance'
def get_waypoint(home, id, n):
    if instance is None:
        return map(add, home[:3], (random.uniform(0.0001, 0.001), random.uniform(0.0001, 0.001), DEFAULT_TAKEOFF))
    else:
        return list(instance.get_waypoint(id, n)["waypoint"])   #list cast is required - JSON doesn't do Tuples
# similar to above, except it gets the speed
def get_speed_to_waypoint(id, n):
    if instance is None:
        return 5
    else:
        return instance.get_waypoint(id, n)["speed_to"]

plan_options = {
    '_order': ['waypoint', 'speed_to'],
    'waypoint': {
        'prompt': "Enter Waypoint (enter for default, 3-tuple directly, 'n' for detailed prompt)",
        'type': [[list, 3, float], str],    # prioritized list of parse options
        'default': lambda home, id, n: get_waypoint(home, id, n),
        'default_args': 'arg:home, arg:id, arg:_n',
        'suboptions': position_options,
        'suboption_args': 'arg:home as home, arg:id as id, arg:_n as num_waypoint',
        'return': 'dict_to_list',
        'return_params': {'order': ['lat', 'lng', 'alt']} #extract returned dict using this order of keys. Remember use dict[key].return
    },
    'speed_to': {
        'prompt': "Enter speed to go to waypoint at, m/s (default 5)",
        'type': float,
        'default': lambda id, n: get_speed_to_waypoint(id, n),
        'default_args': 'arg:id, arg:_n'
    }
}


def get_default_home(vehicle):
    if instance is None:
        return  map(add, DEFAULT_HOME, (random.uniform(0.0001, 0.001), random.uniform(0.0001, 0.001), 0, 0))
    else:
        print instance.get_home(vehicle)
        return list(instance.get_home(vehicle))

per_vehicle_options = {
    '_order': ['instance', 'home', 'name', 'master', 'sitl_port', 'extra_out_ports', 'flight_plan'],
    'instance': {
        'prompt': '  Instance number (default: enumerated automatically)',
        'type': int,
        'default': lambda inst: inst,
        'default_args': 'arg:_n' # apply argument 'instance' to lambda
    },
#NOTE: the default here never gets used. TODO/BUG: default needs to be reproduced next level down
    'home': {
        'prompt': '  Enter home location (enter for default, 4-tuple directly, anything else to prompt further)',
        'type': [[list, 4, float], str],    # prioritized list of allowed type options
        'default': lambda vehicle: get_default_home(vehicle),
        'default_args': 'arg:_n',
        'suboptions': home_options,
        'return': 'dict_to_list',
        'return_params': {'order': ['lat', 'lng', 'alt', 'heading']} #extract returned dict using this order of keys. Remember use dict[key].return
    },
    'master': {
        'prompt': '  Master port',
        'type': int,
        'default': lambda instance: 5760 + 10*instance,
        'default_args': 'arg:_n' # use parent argument 'instance'
    },
    'sitl_port': {
        'prompt': '  SITL port',
        'type': int,
        'default': lambda instance: 5501 + 10*instance,
        'default_args': 'arg:_n' # use parent argument 'instance'
    },
    'extra_out_ports': {
        'prompt': '  List extra out ports (comma or space sep)',
        'type': [list, ANY_LENGTH, int], #ANY_LENGTH signals unknown length
        #'default': lambda instance: [5502 + 10*instance],
        'default': lambda instance: [],
        'default_args': 'arg:_n'
    },
    'flight_plan': {
        'prompt': '  Number of operations/waypoints for this vehicles',
        'type': int,
        'default': lambda vehicle: instance.get_num_waypts(vehicle) if instance else 3,
        'default_args': 'arg:_n',
        'suboptions': plan_options,
        'return': 'use_enumerated_dict',
        'suboption_args': 'this:home as home, arg:_n as id'   #pass in sister value of 'home'
    },
    'name': {
        'prompt': '  Aircraft name',
        'type': str,
        'default': lambda instance: 'sim_' + str(instance),
        'default_args': 'arg:_n'
    }   
}

#ROOT!
options = {
    '_order': ['vehicles'],
    'vehicles': {
        'prompt': 'Number of vehicles',
        'type': int,
        'default': lambda: instance.num_vehicles() if instance is not None else 2,
        'suboptions': per_vehicle_options,
        'suboption_args': '',
        'return': 'use_enumerated_dict'
    }
}

#----------- parsing ----------


#   The following function isn't really used when automatically generating anymore...

#checks whether splitting by comma or space works
#then returns the values parsed
# only works for outer tuple or list type
# and inner types basic primitives eg str, float, int, long (python 2)
def extract_using_split(string, required_len, required_outer_type, required_inner_type):
    print string, required_len, required_outer_type, required_inner_type
    comma_list = [x.strip() for x in string.split(',') if len(x.strip()) > 0]
    space_list = [x for x in string.split(' ') if len(x) > 0]
    use_list = None
    result = [] #convert to tuple later if needed

    if len(space_list) == required_len or required_len == ANY_LENGTH:
        use_list = space_list
    if len(comma_list) == required_len or required_len == ANY_LENGTH:
        if use_list != None:
            print("WARN: found both a space and comma based parse of length %d from %s" %(required_len, string))
        use_list = comma_list
    if use_list == None:
        print("Could not parse structure of length %d from %s" %(required_len, string))
        raise Exception

    for val in comma_list:
        try: 
            result.append(apply(required_inner_type, val))
        except Exception:
            print("Wrong types encountered: parsing %s into %s of %s 's" %(string, str(required_outer_type), str(required_inner_type)))
            raise Exception
    if required_outer_type == tuple:
        return tuple(result)
    else:
        return result

# matches types
def match_simple_type(value, t):
    if t in (str, int, float, long):
        return apply(t, value.strip())
    else:
        if type(t) == str:
            return value == t
        else:
            print("\nHELP")
            raise Exception

#returns pair (value, skip_suboptions)
# handles the prompt to the user, the expected type, how to extract the input
# bit messy but works
def handle_prompt(prompt, expected_type):
    while True:
        value = raw_input(" %s> " %prompt)
        if len(value) == 0:     #signal to use default
            return ('', False)
        try:
            if type(expected_type) == list and list not in [type(elem) for elem in expected_type]:
                # turn something like [list, 4, float] into [[list, 4, float]]
                # ie only one accepted type (ommitting outside [] is just a notation convenience)
                expected_type = [expected_type]
            if type(expected_type) == list:
                #attempt to match any of the options
                for allowed_type in expected_type:
                    if type(allowed_type) == list:
                        try:
                            expected_length = allowed_type[1] #list or tuple
                            if value.strip().startswith('[') or value.strip().startswith('('):
                                # try to check if they entered [1,2,3] or (1,2,3) syntax directly
                                v = ast.literal_eval(value) #cool safe equivalent to eval()!
                                #TODO missing a check here for types within the list/tuple
                                if type(v) != allowed_type[0] or (expected_length != ANY_LENGTH and len(v) != expected_length):
                                    raise Exception
                            else:
                                v = extract_using_split(value, *allowed_type)
                            return (v, True) #if it's a good parse, we can exit and skip suboptions
                        except Exception as e:
                            print str(e)
                    else:
                        return (match_simple_type(value.strip(), allowed_type), False) #skip suboptions!
            else:
                #non-compount type ie just a float, string etc. Handled elsewhere in case it's a string to match
                return (match_simple_type(value.strip(), expected_type), False) #if it's a good parse, we can exit
        except Exception as e:
            print(str(e))


# following two functions can probably be refactored into one

# extracts an argument list in the options to actual python values
# eg "arg:_n" => [value...]
# needed is the required arguments string, args is the args passed in from the parent, and this is the current values collected on this parse level
def get_args_list(needed, args, this):
    if len(needed) == 0:
        return []
    needed_args = needed.split(',')
    extracted = []
    for a in needed_args: #***order matters***
        where, arg = a.split(':')
        #val_name, val_as = arg_val_as.split(' as ')
        if where.strip() == 'arg':
            extracted.append(args[arg.strip()])
        elif where.strip() == 'this':
            extracted.append(this[arg.strip()])
        else:
            print("Unknown 'where' type: %s" %(where))
    return extracted

# same as above, but returns a dict ie. "arg:_n" => {"_n": whatever}
def get_args_dict(needed, args, this):
    if len(needed) == 0:
        return {}
    needed_args = needed.split(',')
    extracted = {}
    for a in needed_args:
        where, arg_val_as = a.split(':')
        if ' as ' not in arg_val_as:
            print("Missing 'as' claus in %s from arg identifier %s" %(arg_val_as, a))
        val_name, val_as = arg_val_as.split(' as ')
        if where.strip() == 'arg':
            extracted[val_as.strip()] = args[val_name.strip()]
        else:
            extracted[val_as.strip()] = this[val_name.strip()]
    return extracted

def get_required_return_type(r):
    s = r.split("_")
    if s[-1] == 'list':
        return list
    elif s[-1] == 'dict':
        return dict
    else:
        print("unknown return type? %s" %r)
        return None


def get_default(spec, args, this):
    #print args, this
    #lambda requires evaluating to get args
    if type(spec['default']) == type(lambda: 1):
        required_args = '' if 'default_args' not in spec else spec['default_args']
        args = get_args_list(required_args, args, this)
        return spec['default'](*args)
    else:
        return spec['default']

# convert a returned dictionary from a suboption into an array according to an order
def result_dict_convert(d, params, to_tuple=False):
    t = []
    if 'order' in params:
        order = params['order']
        for key in order:
            t.append(d[key])
    if to_tuple:
        return tuple(t)
    return t


#messy logic to handle all the parsing
#args has one special key _n which is the enumator value
def experiment_setup_parser(root, args):
    order = root['_order'][:] #copy
    result = {}

    while len(order) > 0:
        key = order.pop(0)
        result[key] = None
        expected_type = root[key]['type']
        prompt = root[key]['prompt']
        if not USE_ALL_DEFAULTS:
            value, skip_suboptions = handle_prompt(prompt, expected_type)
        if USE_ALL_DEFAULTS or value == '': #signals use default
            value = get_default(root[key], args, result)
            if 'return' in root[key]:
                required_type = get_required_return_type(root[key]['return'])
                if type(value) == required_type:
                    skip_suboptions = True
                else:
                    skip_suboptions = False
            else:
                skip_suboptions = True

        if 'suboptions' not in root[key] or skip_suboptions:
            result[key] = value
        else:
            if 'return' not in root[key]:
                raise Exception("Error in formatting: suboptions given without 'return'")
            return_action = root[key]['return']
            suboption_args = get_args_dict(root[key]['suboption_args'], args, result) if 'suboption_args' in root[key] else {}
            if root[key]['return'] == 'use_enumerated_dict':
                result[key] = {}
                for i in range(value):
                    #insert the special _n value
                    suboption_args['_n'] = i
                    result[key][i] = experiment_setup_parser(root[key]['suboptions'], suboption_args)    
            elif root[key]['return'] == 'dict_to_tuple':
                suboption_result = experiment_setup_parser(root[key]['suboptions'], suboption_args)
                params = root[key]['return_params']
                result[key] = result_dict_convert(suboption_result, params, to_tuple=True)
            elif root[key]['return'] == 'dict_to_list':
                suboption_result = experiment_setup_parser(root[key]['suboptions'], suboption_args)
                params = root[key]['return_params']
                result[key] = result_dict_convert(suboption_result, params, to_tuple=False)               
            else:
                print("Do not know what to do with value returned by suboption, unknown: %s" %root[key]['return'])

    return result




if __name__ == '__main__':

    print("""RIGHT these arguments are bloody confusing
      use "-d -ct Hover_Line 5 30 3" to automatically generate a Hover_LineInto scenario with 5 vehicles, initial mean distance 30m and stddev 3m
      use -i or nothing to use the prompts manually
      use -i -d to do a sort of random point to point generation with 2 vehicles
      don't use -i and -ct together because something will break/not work properly...
    """)
    parser = argparse.ArgumentParser(description='Create an experiment automatically')
    mgroup = parser.add_mutually_exclusive_group()
    mgroup.add_argument("-i", action='store_true', help="Interactive prompt for manual entry (default if no arugments provided)")
    mgroup.add_argument("-d", action='store_true', help="Use defaults (overrides manual)")
    parser.add_argument("-ct", nargs = '*', help="Collision type", dest='collision_type')
    
    args = parser.parse_args()

    collision_types = {
        'Hover_Line': Hover_LineInto
    }

    # not interactive and not programmatic => randomly generated with default values
    if not args.i and not args.collision_type:
        USE_ALL_DEFAULTS = True
        instance = None
    # not default and not programmatic collision => purely manual
    elif not args.d and not args.collision_type:
        USE_ALL_DEFAULTS = False
        instance = None
    # generate framework stuff by defaults and all the programmatic collision with defaults
    elif args.collision_type:
        if args.i:
            USE_ALL_DEFAULTS = False
        else:
            USE_ALL_DEFAULTS = True
        ct = args.collision_type
        instance_args = ct[1:]
        instance = collision_types[ct[0]](*instance_args)


    plan = experiment_setup_parser(options, {})

    existingExperimentNums = [int(exp.split('_')[1]) for exp in glob.glob('./experiments/experiment_*')]
    existingExperimentNums.sort()
    try:
        nextNum = existingExperimentNums[-1] + 1
    except Exception: #any sort of error
        nextNum = 0

    experiment_name = "experiment_" + str(nextNum)
    #set up directories for this experiment
    os.mkdir(os.path.join('.', 'experiments', experiment_name))
    open(os.path.join('.', 'experiments', experiment_name, \
        'plan.json'), 'w').write(json.dumps(plan, indent=4))
    print "Sucessfully generated plan " + experiment_name