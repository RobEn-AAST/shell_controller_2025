import carla
import random

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


def spawn_traffic(client, num_vehicles=20, num_pedestrians=30):  
    """  
    Spawns vehicles and pedestrians with proper CARLA API usage  
    client: carla.Client object connected to the server  
    """  
    world = client.get_world()  
    traffic_manager = client.get_trafficmanager(8001)  
    settings = world.get_settings()  
      
    try:  
        # Configure traffic manager  
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)  
        traffic_manager.set_random_device_seed(42)  
        traffic_manager.set_synchronous_mode(True)  
          
        # ========== VEHICLES ==========  
        blueprints = world.get_blueprint_library().filter('vehicle.*')  
        spawn_points = world.get_map().get_spawn_points()  
          
        # Filter out problematic blueprints (e.g., non-4-wheeled vehicles)  
        blueprints = [bp for bp in blueprints if int(bp.get_attribute('number_of_wheels')) == 4]  
          
        # Randomize spawn points  
        random.shuffle(spawn_points)  
          
        vehicles = []  
        for i in range(min(num_vehicles, len(spawn_points))):  
            bp = random.choice(blueprints)  
            transform = spawn_points[i]  
              
            # Set color if vehicle  
            if bp.has_attribute('color'):  
                color = random.choice(bp.get_attribute('color').recommended_values)  
                bp.set_attribute('color', color)  
              
            # Set role_name for autopilot vehicles  
            bp.set_attribute('role_name', 'autopilot')  
  
            vehicle = world.try_spawn_actor(bp, transform)  
            if vehicle:  
                vehicles.append(vehicle)  
          
        # Batch enable autopilot and set speed  
        # The commented out section in your original code is the right place for this.  
        # traffic_manager.global_percentage_speed_difference(30.0) # This sets a percentage difference from speed limit  
          
        # Set desired speed for each vehicle  
        desired_speed_kmh = 20.0 # Your desired speed in km/h  
          
        for vehicle in vehicles:  
            vehicle.set_autopilot(True, traffic_manager.get_port())  
            # Set desired speed for the vehicle using the traffic manager  
            traffic_manager.set_desired_speed(vehicle, desired_speed_kmh)  
            # You can also set a percentage speed difference if you prefer  
            # traffic_manager.vehicle_percentage_speed_difference(vehicle, 0.0) # 0% difference means exactly speed limit  
          
        # ========== PEDESTRIANS ==========  
        pedestrian_blueprints = world.get_blueprint_library().filter('walker.pedestrian.*')  
        controller_bp = world.get_blueprint_library().find('controller.ai.walker')  
          
        pedestrians = []  
        controllers = []  
          
        for _ in range(num_pedestrians):  
            spawn_location = world.get_random_location_from_navigation()  
            if spawn_location:  
                bp = random.choice(pedestrian_blueprints)  
                transform = carla.Transform(spawn_location, carla.Rotation())  
                  
                pedestrian = world.try_spawn_actor(bp, transform)  
                if pedestrian:  
                    pedestrians.append(pedestrian)  
                      
                    # Create AI controller  
                    controller = world.spawn_actor(controller_bp, carla.Transform(), pedestrian)  
                    controllers.append(controller)  
          
        # Start pedestrian movement  
        for controller in controllers:  
            controller.start()  
            controller.go_to_location(world.get_random_location_from_navigation())  
            controller.set_max_speed(1.5)  # 1.5 m/s ~ 5.4 km/h  
          
        # Set synchronous mode if needed  
        settings.synchronous_mode = True  
        settings.fixed_delta_seconds = 0.05  
        world.apply_settings(settings)  
          
        return vehicles, pedestrians, controllers  
          
    except Exception as e:  
        print(f"Traffic spawn failed: {str(e)}")  
        world.apply_settings(settings)  
        raise


def spawn_car_at(client, x=217.56, y=-1.80, z=0.00, autopilot=False):
    """
    Spawns vehicles and pedestrians with proper CARLA API usage
    client: carla.Client object connected to the server
    """
    world = client.get_world()
    traffic_manager = client.get_trafficmanager(8000)
    settings = world.get_settings()
    
    try:
        # Configure traffic manager
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        traffic_manager.set_random_device_seed(42)
        traffic_manager.set_synchronous_mode(True)
        
        # ========== VEHICLES ==========
        blueprints = world.get_blueprint_library().filter('vehicle.*')
        
        # Filter out problematic blueprints
        blueprints = [bp for bp in blueprints if int(bp.get_attribute('number_of_wheels')) == 4]
        
        bp = random.choice(blueprints)
        transform = carla.Transform(carla.Location(x, y, z), carla.Rotation())
        
        # Set color if vehicle
        if bp.has_attribute('color'):
            color = random.choice(bp.get_attribute('color').recommended_values)
            bp.set_attribute('color', color)
        
        vehicle = world.try_spawn_actor(bp, transform)
        
        if autopilot:
            traffic_manager.global_percentage_speed_difference(30.0)  # 30% slower than speed limit
            for vehicle in vehicles:
                vehicle.set_autopilot(True, traffic_manager.get_port())
        
        
        # Set synchronous mode if needed
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        
        return vehicle
        
    except Exception as e:
        print(f"Traffic spawn failed: {str(e)}")
        world.apply_settings(settings)
        raise  # Better CPU performance