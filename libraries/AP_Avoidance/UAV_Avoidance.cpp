#include "UAV_Avoidance.h"
#include <stdio.h>

#define IGNORE_OVERLAP_TIME_MS 250  // if a collision zone is less than this long, ignore
                                    // this could be a major issue if the planes are just moving fast...


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
    1. predict collisions of this vehicle with all other vehicles, generating a bunch of 
        time ranges where the vehicle is too close to others => create Collision_Zones

    2. merge these pairwise Conflict_Zones
        -sort the array/vector/LL of Conflict_Zones by start time
        -take the first element, merge with all the following zones that have time overlap
        -repeat this until all the segments are 'flattened' and overlapping zones
            have been broken into smaller merged zones
            (how to do this exactly seems quite tough, eg one merge could create new zones which need to be checked again)

        Should now a series of Conflict Zones for this vehicle

    [this is the harder step]
    3. For each conflict zone CZ, take each other plane P in CZ
        do a forward extrapolation for that plane P to find its conflict zones
        ***we are only concerned with other conflict zones that overlap 
        the time interval of CZ
        (not including the plane we already have a conflict with, this would already be in step 2)
        -Flatten these conflict zones as in step 2, then apply step 3 recursively
        - on the way up, flatten with the prior result
        - at the top level, merge with results from 2.

    We should now have a complete sequence of conflict zones with all associated vehicles
    for our Aircraft!

    The next conflict zone will dictate what height to fly at based on unique ID's

    Notes:
        -This is all expensive: 
            m conflict zones -> linear time merge with each other
            recursive merging: n aircraft => n*m 



    

    Since this is recalculated often (as soon as we/others send ADSB msg), we want to maximise reuse if possible
    If no new data from ads-b, continue handling avoidance based on current data/collision zone
    
    !!How to take into account height traveling at before entering new conflict zone
        => for now just assume we recalculate often enough this is taken into account
        (matters since deconflict same height into one vertical 'cell' and unique height = unique 'cell')
    
    */

    */



        
}


void UAV_Avoidance::handle_avoidance(AP_Avoidance::Obstacle *threat) {

}


//constructor
UAV_Avoidance::Conflict_Zone::Conflict_Zone(uint32_t start_time,
                                            uint32_t end_time) {
    time_range.t0 = start_time;
    time_range.t1 = end_time;
}

~UAV_Avoidance::Conflict_Zone::Conflict_Zone() {
    // not 100% sure if this needs to be cleared but just do it anyway
    backpointers.clear();
    // rest should be handled by destructing process    
}

void UAV_Avoidance::Conflict_Zone::add_obstacle(Obstacle_Wrapper *obj) {
    backpointers.insert(obj);
}





/*
    Whenever this merge is called and returns TRUE
    it has created/reduced/inserted conflict zones for all Obstacle_Wrappers that care
    This means that the original two conflict zones may have been reduced 
    to a very small range and thus should be deleted.
    Hence, after merge is called from outside this class,
    both zone1 and zone2 should be checked for > IGNORE_OVERLAP_TIME_MS
    and if failing this test zone1/2.remove_this_from_obstacles() should be called
        then call delete zone
        this could really be handled more elegantly with smart pointers
        but out of time!
*/
bool UAV_Avoidance::Conflict_Zone::merge(UAV_Avoidance::Conflict_Zone &zone) {
    UAV_Avoidance::Range intersection = time_range.intersect(zone.get_time_range());

    // if intersection is empty or too short, ignore and return
    if (!intersection.valid || intersection.t1 - intersection.t0 < IGNORE_OVERLAP_TIME_MS) {
        return false;
    }

    //intersection aligned with start of this range
    if (intersection.t0 == valid_range.t0) {
        if (intersection.t1 == valid_range.t1) {
            // intersection is equal to this zone's full range
            // so we have |--r2-¦=r1=¦-|

            //upgrade this zone to contain all obstacles
            // also shorten it's range
            add_obstacles_from(zone);
            time_range.set(intersection);
            
            //shorten other zone's range up to intersection.t0
            // there's a possibility this will shorten it's range to be too small
            // thus the zone needs to be removed
            // this is handled elsewhere
            zone.set_end_time(intersection.t0);

            //create new zone to split other zone into
            // init with correct range
            // if remaining time is too small, don't create the new zone
            if (zone.get_end_time() - intersection.t1 > IGNORE_OVERLAP_TIME_MS) {
                UAV_Avoidance::Conflict_Zone *new_zone = new UAV_Avoidance::Conflict_Zone(intersection.t1, zone.get_end_time());
                new_zone->add_obstacles_from(zone);
            } else {
                UAV_Avoidance::Conflict_Zone *new_zone == nullptr;  //will now be ignored
            }

            // only need to update other zone's LL for now
            // iterate over relevant Obstacle Wrappers that have pointers to other zone
            // after other zone, insert pointer to upgraded merged zone followed by new zone
            // which is remainder of the other zone originally passed in
            for (const auto &ptr : zone.get_obstacles()) {
                ptr->add_zones_after(&zone, &this, new_zone);   //after, insert 1, insert 2
            }

        } else {
            //intersection's t1 is less than t1 then t1 must == other zone's t1
            // this means it's |---r2-¦==|-r1--¦

            UAV_Avoidance::Conflict_Zone *new_zone = new UAV_Avoidance::Conflict_Zone(intersection.t0, intersection.t1);
            // compound the obstacles
            new_zone.add_obstacles_from(this);
            new_zone.add_obstacles_from(zone);

            //adjust other zones ranges to exclude the new intersection
            time_range.t1 = intersection.t0;
            zone.set_start_time(intersection.t1);

            // add the merged/compound zone before this adjusted zone for everyone that cares
            for (const auto &ptr : backpointers) {
                ptr->add_zone_before(&this, new_zone); // add new zone before this
            }
            for (const auto &ptr : zone.get_obstacles()) {
                ptr->add_zone_after(&zone, new_zone);  // add new zone after zone
            }

        }
    } else {    // (intersection.t0 == zone.get_start_time())
        if (intersection.t1 == zone.get_end_time()) {
            //intersection is equal to other zone's full range
            // ie. ¦--r1-|=r2=|-¦

            //upgrade other zone to contain all objects
            // also shorten it's range to be that of intersection
            zone.set_start_time(intersection.t0);
            zone.set_end_time(intersection.t1);
            zone.add_obstacles_from(this);

            //create new zone to split other zone into
            // init with correct range
            // if remaining time is too small, don't create the new zone
            if (get_end_time() - intersection.t1 > IGNORE_OVERLAP_TIME_MS) {
                UAV_Avoidance::Conflict_Zone *new_zone = new UAV_Avoidance::Conflict_Zone(intersection.t1, get_end_time());
                new_zone->add_obstacles_from(this);
            } else {
                UAV_Avoidance::Conflict_Zone *new_zone == nullptr;  //will now be ignored
            }

            // only need to update other zone's LL for now
            // iterate over relevant Obstacle Wrappers that have pointers to other zone
            // after other zone, insert pointer to upgraded merged zone followed by new zone
            // which is remainder of the other zone originally passed in
            for (const auto &ptr : backpointers) {
                ptr->add_zones_after(&this, &zone, new_zone);   //after, insert 1, insert 2
            }

        } else {
            //intersection's t1 is equal to this zone's end time
            // ie. ¦--r1-|==¦--r2-|

            UAV_Avoidance::Conflict_Zone *new_zone = new UAV_Avoidance::Conflict_Zone(intersection.t0, intersection.t1);
            // compound the obstacles
            new_zone.add_obstacles_from(this);
            new_zone.add_obstacles_from(zone);

            //adjust other zones ranges to exclude the new intersection
            zone.set_end_time(intersection.t0);
            time_range.t0 = intersection.t1;


            // add the merged/compound zone before this adjusted zone for everyone that cares
            for (const auto &ptr : backpointers) {
                ptr->add_zone_after(&this, new_zone); // add new zone before this
            }
            for (const auto &ptr : zone.get_obstacles()) {
                ptr->add_zone_before(&zone, new_zone);  // add new zone after zone
            }
        }
    }
    return true;
}



UAV_Avoidance::Range &UAV_Avoidance::Conflict_Zone::get_time_range() {
    return time_range;
}

std::list<UAV_Avoidance::Obstacle_Wrapper*> &UAV_Avoidance::Conflict_Zone::get_obstacles() {
    return backpointers;
}

// shortcut to copy obstacles from one zone to another
void UAV_Avoidance::Conflict_Zone::add_obstacles_from(Conflict_Zone &zone) {
    // ptr will be reference to a ptr i think??
    for (const auto& ptr: zone.get_obstacles()) {
        backpointers.insert(ptr);
    }
}

void UAV_Avoidance::Conflict_Zone::remove_this_from_obstacles() {
    // this is pretty disgusting : O(m*n)
    for (const auto &ptr : backpointers) {
        ptr->delete_zone(&this);
    }
}



// ---- Obstacle Wrapper ----


// Linear time :(
void UAV_Avoidance::Obstacle_Wrapper::delete_zone(Conflict_Zone *z) {
    // find the pointer to the zone to delete
    auto pos = std::find(conflict_zones.begin(), conflict_zones.end(), z);
    conflict_zone.erase(pos);
}

// Linear time :(
// insert a zone after another in the linked list
void UAV_Avoidance::Obstacle_Wrapper::add_zone_after(
        UAV_Avoidance::Conflict_Zone *after, 
        UAV_Avoidance::Conflict_Zone *z) 
{
    if (z == nullptr) {
        return;
    }

    // find position of 'after'
    auto it = std::find(conflict_zones.begin(), conflict_zones.end(), after);
    if (it != l.end()) {
        it++;   // increment to insert after 'after' instead of before
        l.insert(it, z);
    } else {
        // ??? not found
        ::printf("Did not find conflict_zone to insert after in LL??");
        return;
    }
}

// Linear time :(
void UAV_Avoidance::Obstacle_Wrapper::add_zone_before(
        UAV_Avoidance::Conflict_Zone *before, 
        UAV_Avoidance::Conflict_Zone *z) 
{
    if (z == nullptr) {
        return;
    }

    // find 'before'
    auto it = std::find(conflict_zones.begin(), conflict_zones.end(), before);
    if (it != l.end()) {
        l.insert(it, z);    //insert z in front of 'before' if it's found
    } else {
        // ??? not found
        ::printf("Did not find conflict_zone to insert before in LL??");
        return;
    }
}

// Linear time :(
void UAV_Avoidance::Obstacle_Wrapper::add_zones_after(
        UAV_Avoidance::Conflict_Zone *after, 
        UAV_Avoidance::Conflict_Zone *z1,
        UAV_Avoidance::Conflict_Zone *z2) 
{
    // find 'after'
    auto it = std::find(conflict_zones.begin(), conflict_zones.end(), after);
    if (it != l.end()) {
        if (z1 != nullptr) {
            it++;   // increment to insert after 'after'
            l.insert(it, z1);   // insert first element
        }
        if (z2 != nullptr) {
            it++;
            l.insert(it, z2)    // insert second element
        }
    } else {
        // ??? not found
        ::printf("Did not find conflict_zone to insert after in LL??");
        return;
    }
}