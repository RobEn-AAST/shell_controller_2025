#!/usr/bin/env python3

#### PATH SETTING UP, THIS MUST STAY MOST TOP ####
from pathlib import Path
import sys
import os

base_dir = Path(__file__).resolve().parent
vendor_dir = (base_dir / ".." / "ai_src" / "vendor").resolve()

# Add all vendor directories to sys.path
for path in vendor_dir.iterdir():
    if path.is_dir():
        sys.path.insert(0, str(path))

#### END PATH SETTING ####


import random
import rclpy
from rclpy.node import Node
import carla
import numpy as np
from ai_src.carla_others.agents.tools.misc import get_speed
from ai_src.carla_others.agents.navigation.behavior_agent import BehaviorAgent
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

        carla_host = os.getenv("CARLA_SERVER", "ec2-50-19-120-242.compute-1.amazonaws.com")

        self.client = carla.Client(carla_host, 2000)  # type: ignore
        self.client.set_timeout(20)
        self.world = self.client.get_world()

        if carla_host == "localhost":
            from ai_src.navigator.test_lib.traffic_test import spawn_traffic
            spawn_traffic(self.client, 50, 0)

        self.carla_map = self.world.get_map()
        self.ego_vehicle = None
        total_connect_attempts = 40
        for i in range(total_connect_attempts):
            try:
                self.ego_vehicle = next(v for v in self.world.get_actors().filter("vehicle.*") if v.attributes.get("role_name") == "ego_vehicle")
            except Exception:
                self.get_logger().info(f"attempt {i}/{total_connect_attempts} failed, retrying in 1 scond")
                time.sleep(1)

        assert self.ego_vehicle is not None, f"Ego vehcle can't be none! aboriting {self.world.get_actors().filter('vehicle.*')}"

        target_points = [
            [184.73764038085938, -2.0412847995758057, 0.0],
            [161.01739501953125, -2.043816328048706, 0.0],
            [122.09466552734375, -2.0479705333709717, 0.0],
            [59.701297760009766, -2.035862445831299, 0.0],
            [7.788344860076904, -1.9310301542282104, 0.0],
            [177.55947875976562, -2.042050838470459, 0.0],
            [88.47736358642578, -2.045240879058838, 0.0],
            [92.35310363769531, -2.046504020690918, 0.0],
            [290.5014953613281, -2.02996826171875, 0.0],
            [153.8663330078125, -2.0445797443389893, 0.0],
            [267.6562194824219, -2.0324349403381348, 0.0],
            [386.5950927734375, -1.9345017671585083, 0.0],
            [338.9637451171875, -2.0042386054992676, 0.0],
            [334.8652648925781, -2.006415843963623, 0.0],
        ]

        target_locations = [
            carla.Location(x=point[0], y=point[1], z=point[2])
            for point in target_points
        ]

        # ======= MOVE VEHICLE ========
        self.agent = BehaviorAgent(self.ego_vehicle, behavior="normal")  # cautious, normal, aggressive

        # Initialize waypoint index
        current_waypoint_index = 0

        total_waypoints = len(target_locations)
        # Set initial destination
        destination = target_locations[current_waypoint_index]
        self.agent.set_destination(destination)

        self.get_logger().info("Brain node started...")

        # Thresholds
        self.debug_data_timeout = 30  # seconds
        self.stuck_threshold = 3  # seconds

        # Flags
        self.last_debug_time = time.time()
        self.stuck_timer = 0
        self.overtaking = False
        self.original_lane_id = None

        while True:
            # Get and apply control
            control = self.agent.run_step()  # Auto-generates throttle/brake/steering
            if control is not None:
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
                destination = target_locations[current_waypoint_index]
                self.agent.set_destination(destination)

                # LOGGING INFO FOR DEBUGGING
                self.get_logger().info(f"Moving to waypoint {current_waypoint_index}/{total_waypoints-1}")
                current_location = self.ego_vehicle.get_location()
                speed_kmh = get_speed(self.ego_vehicle)
                self.get_logger().info(
                    f"speed: {speed_kmh}, loc: {current_location.x:.2f}, {current_location.y:.2f}, {current_location.z:.2f} => {destination.x:.2f},{destination.y:.2f},{destination.z:.2f}"
                )

            # DEBUG LOG DATA
            current_time = time.time()
            if current_time - self.last_debug_time >= self.debug_data_timeout:
                current_location = self.ego_vehicle.get_location()
                speed_kmh = get_speed(self.ego_vehicle)
                self.get_logger().info(
                    f"speed: {speed_kmh}, loc: {current_location.x:.2f}, {current_location.y:.2f}, {current_location.z:.2f} => {destination.x:.2f},{destination.y:.2f},{destination.z:.2f}"
                )

                self.last_debug_time = current_time

def main(args=None):
    # start ros node
    rclpy.init(args=args)

    node = Brain()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()