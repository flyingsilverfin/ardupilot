from pymavlink import DFReader
import multiprocessing
from multiprocessing.dummy import Pool as ThreadPool
import glob
import os
from math import radians, cos, sin, asin, sqrt, pi


#DEGTORAD = pi/180  #deprecated 
# RF is the eccentricity of the WGS84 ellipsoid
# RF = 298.257223563
# e2 = 1.0 / RF * (2.0 - 1.0 / RF)
# A is the radius of the earth in meters

EARTH_RAD = 6378137.0

# two samples within this constant are accepted as 'near enough' for distance comparison
# since no two samples are at exactly the same time
OK_TIME_DELTA = 50000   #50ms in us


class DFGPSConverter():
    def __init__(self, file):
        # binary files need to be opened with a different class, this is what will likely be used
        if file.lower().endswith('.bin'):   #.lower necessary!
            df = DFReader.DFReader_binary(file)
        else:
            df = DFReader.DFReader_text(file)
        self.df = df
        self.result = []    

    
    def convert(self):
        self.df._rewind()        #reset to ensure at start of log
        while True:
            m = self.df.recv_match(type=['SIM'])    #these messages contain sim state including true position (GPS messages contain error)
            if m is None:
                break
            #extract usec time, and position
            tup = (m.TimeUS, (m.Lat, m.Lng, m.Alt))
            #save these values
            self.result.append(tup)


    def get_data(self):
        return self.result

    def is_converted(self):
        return len(self.result) > 0

    def num_samples(self):
        return len(self.result)

#Presents the above DFGPSConverter's extracted data as a queue
class ParsedLogAdapter():
    def __init__(self, dfgps):
        self.data = dfgps
        if not self.data.is_converted():
            self.data.convert()

        self.next_index = 0
        self.max_index = self.data.num_samples()

    #take the next sample off
    def take(self):
        self.next_index += 1
        if self.next_index > self.max_index:
            return None
        else:
            return self.data.get_data()[self.next_index - 1]
    
    #peek the next timestamp
    def next_time(self):
        return self.data.get_data()[self.next_index][0]

    #peek the next coordinate
    def next_coord(self):
        return self.data.get_data()[self.next_index][1]

    def samples_remaining(self):
        return self.max_index - self.next_index


# Handles a single vehicle's log
class VehicleAnalyzer():
    def __init__(self, id, directory, do_setup=True):
        self.vehicle_directory = directory
        
        #TODO this could be made case insensitive...
        log_files = glob.glob(os.path.join(directory, 'logs', '*.BIN'))

        if len(log_files) > 1:
            print("More than one matching log file! Using last in alphanum sort")
        if len(log_files) == 0:
            print("Didn't find any log files?")
        log_files.sort()
        self.filename = log_files[-1]

        self.id = id
        self.closest_approaches = {}

        #this can be executed later in parallel 
        if do_setup:
            self.do_setup()

    def do_setup(self):
        data_converter = DFGPSConverter(self.filename)
        self.data_adaptor = ParsedLogAdapter(data_converter)


    #take next sample
    def take(self):
        return self.data_adaptor.take()

    #peek next time
    def next_time(self):
        return self.data_adaptor.next_time()


    #compare this vehicle's next sample versus another vehicle's
    def compare(self, source_id, sample):
        t = sample[0]
        #skip if the time difference is too great
        if not VehicleAnalyzer.acceptable_time_delta(t, self.data_adaptor.next_time()):
            return

        #short circuit comparison if we don't know other vehicle at all yet
        accept_immediately = False
        if source_id not in self.closest_approaches:
            self.closest_approaches[source_id] = {}
            accept_immediately = True
        
        #check dist between gps coordinates & altitude
        dist = VehicleAnalyzer.distance(sample[1], self.data_adaptor.next_coord())
        if accept_immediately or dist < self.closest_approaches[source_id]['dist']:
            self.closest_approaches[source_id]['dist'] = dist
            self.closest_approaches[source_id]['time'] = t
            self.closest_approaches[source_id]['this_mav_loc'] = self.data_adaptor.next_coord()
            self.closest_approaches[source_id]['other_mav_loc'] = sample[1]
    
    def has_another_sample(self):
        return self.data_adaptor.samples_remaining() > 0

    def samples_remaining(self):
        return self.data_adaptor.samples_remaining()

    def get_analysis(self):
        return self.closest_approaches


# a bunch of attempted distance functions that weren't very good

    # see https://stackoverflow.com/questions/1108965/taking-altitude-into-account-when-calculating-geodesic-distance
    # @staticmethod   
    # def WGS84ToECEF(point):
    #     outpoint = [0,0,0]
    #     lat = point[1] * DEGTORAD
    #     lon = point[0] * DEGTORAD
    #     sinLat = math.sin(lat)
    #     cosLat = math.cos(lat)
    #     chi = EARTH_RAD /(1 - e2 * sinLat * sinLat)**0.5
    #     outpoint[0] = (chi + point[2]) * cosLat * math.cos(lon)
    #     outpoint[1] = (chi + point[2]) * cosLat * math.sin(lon)
    #     outpoint[2] = (chi * (1 - e2) + point[2]) * sinLat
    #     return outpoint

    # # see same stackoverflow link
    # @staticmethod
    # def distance(p1, p2):
    #     a = VehicleAnalyzer.WGS84ToECEF(p1)
    #     b = VehicleAnalyzer.WGS84ToECEF(p2)
    #     return ((a[0]-b[0])**2 + (a[1] - b[1])**2 + (a[2] - b[2])**2)**0.5

    # # this approach is used in Dronekit or ArduPilot somewhere... not very accurate
    # @staticmethod
    # def distance(p1, p2):
    #     dlat = p2[0] - p1[0]
    #     dlong = p2[1] - p1[1]
    #     horizontal_dist = math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
    #     dZ = p2[2] - p1[2]
    #     return math.sqrt(horizontal_dist**2 + dZ**2)

    # from https://stackoverflow.com/questions/4913349/haversine-formula-in-python-bearing-and-distance-between-two-gps-points
    # altitude is assumed to be in METERS
    # much better than any of the above methods!!
    @staticmethod
    def distance(p1, p2):
        """
        Calculate the great circle distance between two points 
        on the earth (specified in decimal degrees)
        """
        lat1, lon1, alt1 = p1
        lat2, lon2, alt2 = p2
        # convert decimal degrees to radians 
        lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

        # haversine formula 
        dlon = lon2 - lon1 
        dlat = lat2 - lat1 
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * asin(sqrt(a)) 
        horizontal_dist = c * EARTH_RAD
        return sqrt(horizontal_dist**2 + (alt2-alt1)**2)    # straight line approx works well enough over short distances

    @staticmethod
    def acceptable_time_delta(t1, t2):
        if t2 - t1 < OK_TIME_DELTA:     # 50ms
            return True
        else:
            return False
        

# Encapsulates analysis for an entire set of vehicle logs/one experiment
class ExperimentAnalyzer():
    def __init__(self, experiment_dir):
        # get all vehicle folders
        vehicle_dirs = glob.glob(os.path.join(experiment_dir, 'vehicle_*'))
        self.vehicle_analyzers = {}
        self.num_vehicles = len(vehicle_dirs)
        # 1 thread experimentally turns out to be fastest (since we're waiting on disk?)
        num_threads = 1         
        self.thread_pool = ThreadPool(num_threads)
        for vd in vehicle_dirs:
            p = vd.split('/')
            split = p[-1].split('_')
            id = int(split[1])
            vehicle_dir = os.path.join(experiment_dir, p[-1])
            print vehicle_dir
            self.vehicle_analyzers[id] = VehicleAnalyzer(id, vehicle_dir, do_setup=False)

        #execute setup in parallel if set up to
        self.thread_pool.map(self._setup, self.vehicle_analyzers.keys())

    def _setup(self, vehicle_id):
        print "executing setup for %d" %(vehicle_id)
        self.vehicle_analyzers[vehicle_id].do_setup()

    # a pretty inefficient implementation but it should be ok for now...
    def analyze(self):
        # just compute until any vehicle is out of samples
        # this isn't even accurate but it doesn't matter
        samples_quantities = [v.samples_remaining() for v in self.vehicle_analyzers.values()]
        n = max(samples_quantities)

        # the one we work from of is the one with the earliest timestamp
        # that way there's always something to compare to
        # skip any vehicle that doesn't have a corresponding close sample
        vehicle_id_list = [v.id for v in sorted(self.vehicle_analyzers.values(), key=lambda analyzer: analyzer.next_time())]
        
        first_vehicle = vehicle_id_list[0]

        for _ in range(n): 
            t = self.vehicle_analyzers[first_vehicle].next_time()
            i = 0
            try:
                while i < len(vehicle_id_list):

                    # if this vehicle's timestamp is too far ahead of vehicle 0's
                    # skip it
                    t_prime = self.vehicle_analyzers[vehicle_id_list[i]].next_time()    #peek at next sample's timestamp
                    if t_prime - t > OK_TIME_DELTA:
                        print("Timestamp from vehicle %d is too far ahead: %d vs %d, skipping" %(vehicle_id_list[i], t, t_prime))
                        i += 1
                        continue
                    if t_prime - t < 0:
                        print("Timestamp from vehicle %d is *behind* first vehicle's. THIS SHOULDN'T BE HAPPNING" %(vehicle_id_list[i]))
                        break
                    #compare with all other vehicles
                    data_point = self.vehicle_analyzers[vehicle_id_list[i]].take()
                    for v_id in vehicle_id_list[i+1:]:      #skip the earlier vehicles in the list, they've already compared to this sample
                        #print("Passing sample %d from vehicle %d to %d" %(k, vehicle_id_list[i], v_id))
                        self.vehicle_analyzers[v_id].compare(vehicle_id_list[i], data_point)
                    i += 1
            # continue doing the comparisons until anyone runs out of samples at any point
            # then we abort
            # this is good enough to replace the for _ in range(n) with a while?
            except IndexError:
                break
        
    def get_analysis(self):
        for id in self.vehicle_analyzers:
            v = self.vehicle_analyzers[id]
            analysis = v.get_analysis()
            for other_vehicle in analysis:
                print("%d got within %f of %d" %(id, analysis[other_vehicle]['dist'], other_vehicle))
            print v.get_analysis()     # could do something else here, return it or something

if __name__ == '__main__':
    if len(sys.argv) <= 1:
        print("Need to provide directory of experiment logs (will contain many vehicle_* subdirectories)")
    directory = sys.argv[1]
    analyzer = ExperimentAnalyzer(directory)
    analyzer.analyze()
    analyzer.get_analysis()
