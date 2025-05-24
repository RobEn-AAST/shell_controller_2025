import carla

def xyz_to_locs(wps, carla_map):
    """Convert XYZ coordinates to rightmost drivable lane locations"""
    target_locations = []
    
    for x, y, z in wps:
        # Get initial waypoint projection
        loc = carla.Location(x=x, y=y, z=z) # type: ignore
        wp = carla_map.get_waypoint(loc, project_to_road=True)
        
        target_locations.append(wp.transform.location)
    
    return target_locations
