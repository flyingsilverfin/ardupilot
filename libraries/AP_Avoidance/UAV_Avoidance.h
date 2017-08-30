#pragma once

#include "AP_Avoidance.h"
#include <list>

/*
    This is a friend class of AP_Avoidance that handles the uav avoidance stuff
*/
class UAV_Avoidance {

    public:
        //constructor
        UAV_Avoidance(AP_Avoidance &avoidance);
        void deinit();

        void update(); //todo

        void handle_avoidance(AP_Avoidance::Obstacle *threat); //passing threat may be unncessary

    private:
        const AP_Avoidance &_avoidance;

        // represent this vehicle as an Obstacle as well
        AP_Avoidance::Obstacle *this_vehicle;
        // timestamp of last adsb-in message
        uint32_t last_adsb_in_ms;

        uint16_t maneuver_counter = 0;    //updates when new deconfliction/changes beavhior

        bool update_vehicle_as_obstacle()





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
    } Range;


    class Obstacle_Wrapper {
        public:
            Obstacle_Wrapper(AP_Avoidance::Obstacle *obs);

        private:
            AP_Avoidance::Obstacle obstacle;
            std::list<Conflict_Zone*> conflict_zones; // linked list of upcoming conflict zones

    }

    class Conflict_Zone {
        public:
            Conflict_Zone(Obstacle_Wrapper *obj1, 
                            Obstacle_Wrapper *obj2,
                            uint32_t start_time,
                            uint32_t end_time

            Conflict_Zone *merge(Conflict_Zone zone);

            // retrieve height need to fly at 
            int32_t get_deconflicted_height(uint32_t id);
            // retrieve list of obstacle wrappers for merging
            std::list<Obstacle_Wrapper*> &get_obstacles();

            //set range start time
            void set_start_time(uint32_t t) {
                time_range.t0 = t;
            }

            // set range end time
            void set_end_time(uint32_t t) {
                time_range.t1 = t;
            }



        private:
            Range time_range;
            std::list<Obstacle_Wrapper*> backpointers;  // hold onto all vehicles in this zone

    }

    

}
