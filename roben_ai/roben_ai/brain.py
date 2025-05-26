#!/usr/bin/env python3

#### PATH SETTING UP, THIS MUST STAY MOST TOP ####
from pathlib import Path
import sys
import os

base_dir = Path(__file__).resolve().parent
vendor_dir = (base_dir / '..' / 'ai_src' / 'vendor').resolve()

for pkg_dir in vendor_dir.iterdir():
    if pkg_dir.is_dir():
        sys.path.insert(0, str(pkg_dir))

import random
import rclpy
from rclpy.node import Node
import carla
import numpy as np
from ai_src.carla_others.agents.tools.misc import get_speed
from ai_src.carla_others.agents.navigation.global_route_planner import GlobalRoutePlanner
from ai_src.carla_others.agents.navigation.behavior_agent import BehaviorAgent
from ai_src.navigator.wp_utils import xyz_to_right_lane
from ai_src.navigator.tsp_solver import optimize_route_order
from ai_src.navigator.test import spawn_traffic
import time


# pytorch stablebaseline gymansium
class Brain(Node):
    def __init__(self):
        """
        The main controller of them all, the ultimate god of this body!
        Takes the image, and any other sensors we could find,
        sends them to an actor critic policy, and retreives the action we are asked to execute
        And executes it! voalla!!
        """
        super().__init__("brain")

        carla_host = os.getenv('CARLA_SERVER', 'ec2-50-19-120-242.compute-1.amazonaws.com')  

        self.client = carla.Client(carla_host, 2000) # type: ignore
        self.client.set_timeout(20)
        self.world = self.client.get_world()

        self.carla_map = self.world.get_map()
        self.ego_vehicle = None
        total_connect_attempts = 40
        for i in range(total_connect_attempts):
            try:
                self.ego_vehicle = next(v for v in self.world.get_actors().filter("vehicle.*") if v.attributes.get("role_name") == "ego_vehicle")
            except Exception:
                self.get_logger().info(f"attempt {i}/{total_connect_attempts} failed, retrying in 1 scond")
                time.sleep(1)
            
        target_points = [
            [334.949799,-161.106171,0.001736],
            [339.100037,-258.568939,0.001679],
            [396.295319,-183.195740,0.001678],
            [267.657074,-1.983160,0.001678],
            [153.868896,-26.115866,0.001678],
            [290.515564,-56.175072,0.001677],
            [92.325722,-86.063644,0.001677],
            [88.384346,-287.468567,0.001728],
            [177.594101,-326.386902,0.001677],
            [-1.646942,-197.501282,0.001555],
            [59.701321,-1.970804,0.001467],
            [122.100121,-55.142044,0.001596],
            [161.030975,-129.313187,0.001679],
            [184.758713,-199.424271,0.001680],
        ]
        sampling_resolution = 1.0

        target_locations = xyz_to_right_lane(target_points, self.carla_map)

        grp = GlobalRoutePlanner(self.carla_map, sampling_resolution)

        optimized_targets = optimize_route_order(
            self.ego_vehicle.get_location(),
            target_locations,
            grp,
        )

        
        self.get_logger().info("\n\n\n\nOPTIMIZED TARGETS ORDER START\n\n\n\n")  
        for i, point in enumerate(optimized_targets):
            self.get_logger().info(f"point {i}: x= {point.x}, y={point.y}, z={point.z}")  
        self.get_logger().info("\n\n\n\nOPTIMIZED TARGETS ORDER END\n\n\n\n")  

        # ======= MOVE VEHICLE ========  
        self.agent = BehaviorAgent(self.ego_vehicle, behavior='aggressive')  # cautious, normal, aggressive  
        self.agent.ignore_traffic_lights(True)  
        self.agent._behavior.max_speed = 47
        # self.agent.set_target_speed(47)
        
        # Initialize waypoint index  
        current_waypoint_index = 0  
        
        total_waypoints = len(optimized_targets)  
        # Set initial destination  
        destination = optimized_targets[current_waypoint_index]  
        self.agent.set_destination(destination)  
        
        self.get_logger().info("Brain node started...")  

        # Thresholds
        self.debug_data_timeout = 30 # seconds
        self.stuck_threshold = 3 # seconds

        # Flags
        self.last_debug_time = time.time()
        self.stuck_timer = 0  
        self.overtaking = False  
        self.original_lane_id = None 
        
        while True: 
            self._overtake()
            # Get and apply control  
            control = self.agent.run_step()  # Auto-generates throttle/brake/steering  
            self.ego_vehicle.apply_control(control)  
            
            # Check if we've reached the current destination  
            if self.agent._local_planner.done():  
                # Move to next waypoint  
                current_waypoint_index += 1  
                
                # Check if we've reached the end of our waypoints  
                if current_waypoint_index >= total_waypoints:  
                    self.get_logger().info("All waypoints reached!")  
                    break  
                
                # Set the next destination  
                destination = optimized_targets[current_waypoint_index]  
                self.agent.set_destination(destination)
                
                # LOGGING INFO FOR DEBUGGING
                self.get_logger().info(f"Moving to waypoint {current_waypoint_index}/{total_waypoints-1}")  

            # DEBUG LOG DATA
            current_time = time.time()
            if current_time - self.last_debug_time >= self.debug_data_timeout:
                current_location = self.ego_vehicle.get_location()
                speed_kmh = get_speed(self.ego_vehicle)
                self.get_logger().info(f"speed: {speed_kmh}, loc: {current_location.x:.2f}, {current_location.y:.2f}, {current_location.z:.2f} => {destination.x:.2f},{destination.y:.2f},{destination.z:.2f}")

                self.last_debug_time = current_time
    
    def _overtake(self):
        # Check if vehicle is stuck behind another vehicle  
        vehicle_list = self.world.get_actors().filter("*vehicle*")  
        ego_location = self.ego_vehicle.get_location()  
        ego_waypoint = self.carla_map.get_waypoint(ego_location)  
        
        # Detect if there's a slow/stopped vehicle ahead  
        affected_by_vehicle, blocking_vehicle, distance = self.agent._vehicle_obstacle_detected(vehicle_list, 15.0)  
        
        if affected_by_vehicle and not self.overtaking:  
            # Check if the blocking vehicle is moving slowly  
            blocking_speed = blocking_vehicle.get_velocity().length()  
            if blocking_speed < 1.0:  # Less than 1 m/s  
                self.stuck_timer += 0.05  # Assuming 20 FPS  
                if self.stuck_timer > self.stuck_threshold:  
                    # Initiate overtaking  
                    self.overtaking = True  
                    self.original_lane_id = ego_waypoint.lane_id  
                    # Try left lane first, then right  
                    self.agent.lane_change('left')  
                    self.stuck_timer = 0  
            else:  
                self.stuck_timer = 0  
        elif self.overtaking:  
            # Check if we can return to original lane  
            current_waypoint = self.ego_vehicle.get_world().get_map().get_waypoint(ego_location)  
            if current_waypoint.lane_id != self.original_lane_id: 
                # Check if we've passed the obstacle  
                if not affected_by_vehicle or distance > 20.0:  
                    # Return to original lane  
                    if self.original_lane_id < current_waypoint.lane_id:  
                        self.agent.lane_change('right')  
                    else:  
                        self.agent.lane_change('left')  
                    self.overtaking = False  

def main(args=None):
    # start ros node
    rclpy.init(args=args)

    node = Brain()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()