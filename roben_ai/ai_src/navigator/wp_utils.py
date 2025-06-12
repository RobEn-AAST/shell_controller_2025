import carla

def xyz_to_right_lane(wps, carla_map):
    """Convert XYZ coordinates to rightmost drivable lane locations"""
    target_locations = []
    
    for x, y, z in wps:
        # Get initial waypoint projection
        loc = carla.Location(x=x, y=y, z=z) # type: ignore
        wp = carla_map.get_waypoint(loc, project_to_road=True)
        
        # Find rightmost drivable lane
        current_wp = wp
        # while True:
        #     next_wp = current_wp.get_right_lane()
        #     if not next_wp or next_wp.lane_type not in [carla.LaneType.Driving, carla.LaneType.Shoulder]: # type: ignore
        #         break
        #     current_wp = next_wp
        
        target_locations.append(current_wp.transform.location)
    
    return target_locations
