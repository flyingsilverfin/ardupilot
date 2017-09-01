#pragma once

#include "AP_Avoidance.h"
#include <unordered_set>
/*
    This is a friend class of AP_Avoidance that handles the uav avoidance 
    ***preliminary idea sketching***
*/

/*

    All of this should be rewritten with smart pointers
    Current code is just a few days of brain dumping ideas/skteching precesses as fast as possible

*/
class UAV_Avoidance {

    public:
        //constructor
        UAV_Avoidance(AP_Avoidance &avoidance);
        void deinit();

        void update();

        void handle_avoidance(AP_Avoidance::Obstacle *threat); //passing threat may be unncessary

    private:
        const AP_Avoidance &_avoidance;

        // represent this vehicle as an Obstacle as well
        AP_Avoidance::Obstacle *this_vehicle;
        // timestamp of last adsb-in message
        uint32_t last_adsb_in_ms;

        // uint16_t maneuver_counter = 0;    //updates when new deconfliction/changes beavhior

        bool update_vehicle_as_obstacle()




// implements [t0, t1] and intersections hereof
    typedef struct Range {
        uint32_t t0;
        uint32_t t1;
        bool valid;

        // copy return, small struct
        Range intersect(Range &r) {
            Range ret_range;
            if (t1 < r.t0 || t0 > r.t1) {
                ret_range.valid = false;
                return ret_range;
            }

            if (t0 >= r.t0) {
                ret_range.t0 = t0;
            } else {
                ret_range.t0 = r.t0;
            }

            if (t1 <= r.t1) {
                ret_range.t1 = t1;
            } else {
                ret_range.t1 = r.t1;
            }

            return ret_range;
        }

        void set(Range &r) {
            t0 = r.t0;
            t1 = r.t1;
        }
        uint32_t length() {
            return t1 - t0;
        }
    } Range;


    // these are not new'ed but live within UAV_Avoidance
    // so they don't need to be smart pointered?
    class Obstacle_Wrapper {
        public:
            Obstacle_Wrapper(AP_Avoidance::Obstacle *obs);

            void add_zone_after(Conflict_Zone* after, Conflict_Zone *z);
            void add_zone_before(Conflict_Zone* before, Conflict_Zone *z);
            void add_zones_after(Conflict_Zone *after, Conflict_Zone *z1, Conflict_Zone *z2);

            void delete_zone(Conflict_Zone *z);

        private:
            AP_Avoidance::Obstacle obstacle;

            // LL the best to go for here?
            // tradeoff between inserting/finding
            //  these should definitely be smart pointers since Conflict Zones are new'ed
            std::list<Conflict_Zone*> conflict_zones;

    }


    // these get NEW'ed on the heap
    // be sure to delete them on update...
    // or use smart pointers as I mention at the top of this header file
    class Conflict_Zone {
        public:
            //normal constructor
            Conflict_Zone(uint32_t start_time,
                            uint32_t end_time);
            ~Conflict_Zone();

            //merge two conflict zones and handle all the remappinp
            bool merge(Conflict_Zone zone); // bool is success/ignored

            void remove_this_from_obstacles();

            //add obstacles to list
            void add_obstacle(Obstacle_Wrapper *obj);
            void add_obstacles_from(Conflict_Zone &zone);

            // retrieve height need to fly at 
            int32_t get_deconflicted_height(uint32_t id);

            // retrieve set of obstacle wrappers for merging
            std::unordered_set<Obstacle_Wrapper*> &get_obstacles() {
                return backpointers;
            }

            //range getters/setters
            void set_start_time(uint32_t t) {
                time_range.t0 = t;
            }
            void set_end_time(uint32_t t) {
                time_range.t1 = t;
            }
            uint32_t get_start_time() {
                return time_range.t0;
            }
            uint32_t get_end_time() {
                return time_range.t1;
            }
            // t1 - t0 basically
            uint32_t get_length() {
                return time_range.length();
            }



        private:
            Range time_range;
            // hold onto all vehicles in this zone in a unique set for easy merging
            std::unordered_set<Obstacle_Wrapper*> backpointers;  

    }

    

}
