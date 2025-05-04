import carla

def validate_route_lanes(full_route):
    right_lane_count = 0
    total_waypoints = 0
    
    for wp, _ in full_route:
        # Get current lane's right boundary
        right_lane_wp = wp.get_right_lane()
        
        # Check if already in rightmost lane
        if not right_lane_wp or right_lane_wp.lane_type not in [carla.LaneType.Driving, carla.LaneType.Shoulder]: # type: ignore
            right_lane_count += 1
            
        total_waypoints += 1
        
    print(f"Right lane adherence: {right_lane_count/total_waypoints:.1%}")
