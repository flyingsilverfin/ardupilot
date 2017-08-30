#include "UAV_Avoidance.h"



UAV_Avoidance::UAV_Avoidance(AP_Avoidance &avoidance) :
    _avoidance(avoidance) 
{
    this_vehicle = new Obstacle();
}


bool update_vehicle_as_obstacle() {
    adsb_vehicle_t *data = _avoidance.adsb.last_out_data();

    // TODO 
    // I'm pretty sure last_update_ms is the time between ADSB messages
    // not an absolute timestamp... it will likely be different
    // but not a monotonically increasing value!
    // is the use in other parts of AP_Avoidance correct?

    // if the last adsb message sent is still the same, skip
    if (data->last_update_ms == this_vehicle->timestamp_ms) {
        return false;
    }
    this_vehicle->timestamp_ms = data->last_update_ms;

    mavlink_adsb_vehicle_t &info = data->info;  //shortcut
    Location &loc = this_vehicle->_location;    //shortcut
    loc.alt = info.altitude*0.1f;   //in cm like other obstacles
    loc.lat = info.lat;
    loc.lng = info.lon;

    Vector3f &vel = this_vehicle->_velocity;    //shortcut
    vel.z = -0.01f*info.ver_velocity;  //positive is now down, m/s

    float hor_vel = info.hor_velocity * 0.01f;   //cm/s -> m/s
    float heading = info.heading * 0.01f;    //centidegrees -> deg

    float vel_east = hor_vel * sin(radians(heading));
    float vel_north = hor_vel * cos(radians(heading)));

    vel.x = vel_east;
    vel.y = vel_north;

    return true;    //signal new data
}

/*
    Here we do all the calculations for conflict zones etc
    using the obstacles[] array in AP_Avoidance of which this class is a friend
    we can also access AP_ADSB to and do last_out_data() to get the the data in the last ADSB-Out message
    this way we can restrict ourselves to seeing only the exact same information as everyone else
*/
void UAV_Avoidance::update() {
    bool this_vehicle_updated = update_vehicle_as_obstacle();
    uint32_t last_adsb_in = _adsb.last_adsb_in();
    bool other_vehicles_updated = last_adsb_in == last_adsb_in_ms;

    if (!this_vehicle_updated && !other_vehicles_updated) {
        return;
    }
    last_adsb_in_ms = last_adsb_in; //update the last last time we got an adsb-in message


    /*
    Steps:
    PRETTY BAD EFFICIENCY atm
    not sure how improvable it is
    proof of concept for now
    1. predict collisions of all aircrafts with each other, stick pairs into Conflict Zones
    2. if this is vehicle X, take all Conflict Zones it's involved with, merge overlapping ones => more conflict zones
    3. Recursively for each of these conflict zones, check other aircraft involved in this conflict zone
        for that aircraft merge this conflict zone with other overlapping ones => repeat for resulting ones
            !!somehow send these back to the top aircraft we're interested in?
    
    4. This process should result in a linked list of Conflict Zones for our aircraft
        Each of which contains all aircrafts in the zone at the time
        The conflict zone returns a height we should travel at based on ordering of ID's
    

    Since this is recalculated often (as soon as we/others send ADSB msg), we want to maximise reuse if possible
    If no new data from ads-b, continue handling avoidance based on current data
    
    !!How to take into account height traveling before entering new conflict zone
        => for now just assume we recalculate often enough this is taken into account
        (matters since deconflict same height into one vertical 'cell' and unique height = unique 'cell')
    
    */



        
}


void UAV_Avoidance::handle_avoidance(AP_Avoidance::Obstacle *threat) {

}


//constructor
UAV_Avoidance::Conflict_Zone::Conflict_Zone(uint32_t start_time,
                                            uint32_t end_time) {
    valid_time_range.t0 = start_time;
    valid_time_range.t1 = end_time;
}

UAV_Avoidance::Conflict_Zone::add_obstacle(Obstacle_Wrapper *obj) {
    backpointers.push_back(obj);
}



UAV_Avoidance::Conflict_Zone *UAV_Avoidance::Conflict_Zone::merge(UAV_Avoidance::Conflict_Zone &zone) {
    UAV_Avoidance::Range intersection = time_range.intersect(zone.get_time_range());
    if (!intersection.valid) {
        return nullptr;
    }

    //TODO
    // if intersection is the same range as either conflict zone, one is contained within the other
    // in that case, we just upgrade the smaller range with the union of sets of obstacles
    // and make a new conflict zone which is the second half of the larger range

    // if the intersection is some incomplete overlap of both ranges, create a new zone
    // this is the middle zone
    // reduce ranges of the existing ranges

    // !! This needs to be done for ALL overlapping ranges... might be multiple :(
    // this is turning out to be quite complicated!




    // basic code for one zone containing the other
    UAV_Avoidance::Conflict_Zone *new_zone = new UAV_Avoidance::Conflict_Zone(intersection.t0, intersection.t1);

    for (UAV_Avoidance::Obstacle_Wrapper *ptr : backpointers) {
        new_zone.add_obstacle(ptr);
    }

    std::list<UAV_Avoidance::Obstacle_Wrapper*> &obstacles = zone.get_obstacles();
    for (UAV_Avoidance::Obstacle_Wrapper *ptr: obstacles) {
        new_zone.add_obstacle(ptr);
    }


// TODO
// hard bit... adjust pointers in the Obstacle_Wrappers



}



UAV_Avoidance::Range &UAV_Avoidance::Conflict_Zone::get_time_range() {
    return time_range;
}

std::list<UAV_Avoidance::Obstacle_Wrapper*> &UAV_Avoidance::get_obstacles() {
    return backpointers;
}

