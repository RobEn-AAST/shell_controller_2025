#!/usr/bin/env python3

#### PATH SETTING UP, THIS MUST STAY MOST TOP ####
from pathlib import Path
import sys
import os

base_dir = Path(__file__).resolve().parent


# Add networkx to sys.path
networkx_dir = (base_dir / ".." / "ai_src" / "vendor" / "networkx").resolve()
sys.path.insert(0, str(networkx_dir))

# Add shapely to sys.path
shapely_dir = (base_dir / ".." / "ai_src" / "vendor" / "shapely").resolve()
sys.path.insert(0, str(shapely_dir))

# Add ortools to sys.path
ortools_dir = (base_dir / ".." / "ai_src" / "vendor" / "ortools").resolve()
sys.path.insert(0, str(ortools_dir))

# Add google to sys.path
google_dir = (base_dir / ".." / "ai_src" / "vendor" / "google").resolve()
sys.path.insert(0, str(google_dir))

# Add protobuf to sys.path
protobuf_dir = (base_dir / ".." / "ai_src" / "vendor" / "protobuf").resolve()
sys.path.insert(0, str(protobuf_dir))

# Add casadi
casadi_dir = (base_dir / ".." / "ai_src" / "vendor" / "casadi").resolve()
sys.path.insert(0, str(casadi_dir))

# Add pygame
pygame_dir = (base_dir / ".." / "ai_src" / "vendor" / "pygame").resolve()
sys.path.insert(0, str(pygame_dir))


# Add self_driving
self_driving_dir = (base_dir / ".." / "ai_src" / "self_driving").resolve()
sys.path.insert(0, str(self_driving_dir))

# Add ai_src (can be neglected, helps only debugger mode)
ai_src_dir = (base_dir / "..").resolve()
sys.path.insert(0, str(ai_src_dir))

import rclpy
from rclpy.node import Node
import carla
from ai_src.navigator.wp_utils import xyz_to_locs
from ai_src.carla_others.agents.tools.misc import get_speed
from ai_src.carla_others.agents.navigation.global_route_planner import GlobalRoutePlanner
from ai_src.navigator.tsp_solver import optimize_route_order
from ai_src.navigator.test import spawn_traffic, spawn_car_at
import time
import pygame

import carla

from manual_control import KeyboardControl
from hud import HUD
from ego_vehicle import World


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
        self.map = self.world.get_map()
        self.ego_vehicle = None

        if carla_host == "localhost":
            spawn_car_at(self.client, autopilot=False)

        total_connect_attempts = 40
        for i in range(total_connect_attempts):
            try:
                self.ego_vehicle = next(v for v in self.world.get_actors().filter("vehicle.*") if v.attributes.get("role_name") == "ego_vehicle")
            except Exception:
                self.get_logger().info(f"attempt {i}/{total_connect_attempts} failed, retrying in 1 scond")
                time.sleep(1)

        optimized_targets = [
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

        # sampling_resolution = 1.0
        # target_locations = xyz_to_locs(target_points, self.map)
        # grp = GlobalRoutePlanner(self.map, sampling_resolution)
        # optimized_targets = optimize_route_order(
        #     self.ego_vehicle.get_location(),
        #     target_locations,
        #     grp,
        # )

        # self.get_logger().info("\n\n\n\nOPTIMIZED TARGETS ORDER START\n\n\n\n")
        # for i, point in enumerate(optimized_targets):
        #     self.get_logger().info(f"point {i}: x= {point.x}, y={point.y}, z={point.z}")
        # self.get_logger().info("\n\n\n\nOPTIMIZED TARGETS ORDER END\n\n\n\n")

        # ======= MOVE VEHICLE ========
        pygame.init()
        pygame.font.init()
        display = pygame.display.set_mode((1280, 720), pygame.HWSURFACE | pygame.DOUBLEBUF)
        hud = HUD(1280, 720)

        driver_agent = World(carla_world=self.world, hud=hud, agent_str="Autonomous")

        controller = KeyboardControl(driver_agent, start_in_autopilot=True)

        # Manually start the vehicle to avoid control delay
        driver_agent.player.apply_control(carla.VehicleControl(manual_gear_shift=True, gear=1))
        driver_agent.player.apply_control(carla.VehicleControl(manual_gear_shift=False))

        clock = pygame.time.Clock()

        current_waypoint_index = 0
        total_waypoints = len(optimized_targets)
        # Set initial destination
        destination = optimized_targets[current_waypoint_index]
        driver_agent.agent.set_destination([destination[0], destination[1], destination[2]])

        self.debug_data_timeout = 30  # seconds
        self.last_debug_time = time.time()

        self.get_logger().info("Brain node driving now...")
        while True:
            # Keyboard control
            clock.tick_busy_loop(60)
            if controller.parse_events(self.client, driver_agent, clock):
                return

            # driver_agent.world.wait_for_tick(5.0)
            driver_agent.tick(clock)
            driver_agent.render(display)
            pygame.display.flip()

            # In your main update loop
            driver_agent.front_radar.update()
            driver_agent.left_front_radar.update()
            driver_agent.left_back_radar.update()

            if driver_agent.front_radar.detected:  
                print(f"Front obstacle at relative position: {driver_agent.front_radar.rel_pos}")  
                print(f"Relative velocity: {driver_agent.front_radar.rel_vel}")

            control = driver_agent.agent.run_step(debug=True)

            # Agent autopilot
            if driver_agent.autopilot_mode:
                # control signal to vehicle
                control.manual_gear_shift = False
                driver_agent.player.apply_control(control)

            # Check if we've reached the current destination
            if driver_agent.agent._local_planner.done():
                # Move to next waypoint
                current_waypoint_index += 1

                # Check if we've reached the end of our waypoints
                if current_waypoint_index >= total_waypoints:
                    self.get_logger().info("All waypoints reached!")
                    break

                # Set the next destination
                destination = optimized_targets[current_waypoint_index]
                driver_agent.agent.set_destination([destination[0], destination[1], destination[2]])

                # LOGGING INFO FOR DEBUGGING
                self.get_logger().info(f"Moving to waypoint {destination}, done_count: {current_waypoint_index}/{total_waypoints-1}")

            # DEBUG LOG DATA
            current_time = time.time()
            if current_time - self.last_debug_time >= self.debug_data_timeout:
                speed_kmh = get_speed(self.ego_vehicle)
                current_location = self.ego_vehicle.get_location()

                self.get_logger().info(
                    f"speed: {speed_kmh}, loc: {current_location.x:.2f}, {current_location.y:.2f}, {current_location.z:.2f} => {destination[0]:.2f},{destination[1]:.2f},{destination[2]:.2f}"
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
