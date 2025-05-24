#!/usr/bin/env python3

#### PATH SETTING UP, THIS MUST STAY MOST TOP ####
from pathlib import Path
import sys
import os

base_dir = Path(__file__).resolve().parent

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
from ai_src.navigator.test import spawn_traffic
import time
import pygame

import carla

from manual_control import KeyboardControl
from hud import HUD
from ego_vehicle import World


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
        self.map = self.world.get_map()
        self.ego_vehicle = None

        if carla_host == 'localhost':
            spawn_traffic(self.client, 30, 30)

        total_connect_attempts = 40
        for i in range(total_connect_attempts):
            try:
                self.ego_vehicle = next(v for v in self.world.get_actors().filter("vehicle.*") if v.attributes.get("role_name") == "ego_vehicle")
            except Exception:
                self.get_logger().info(f"attempt {i}/{total_connect_attempts} failed, retrying in 1 scond")
                time.sleep(1)

        target_points = [
            [334.949799, -161.106171, 0.001736],
            [339.100037, -258.568939, 0.001679],
            [396.295319, -183.195740, 0.001678],
            [267.657074, -1.983160, 0.001678],
            [153.868896, -26.115866, 0.001678],
            [290.515564, -56.175072, 0.001677],
            [92.325722, -86.063644, 0.001677],
            [88.384346, -287.468567, 0.001728],
            [177.594101, -326.386902, 0.001677],
            [-1.646942, -197.501282, 0.001555],
            [59.701321, -1.970804, 0.001467],
            [122.100121, -55.142044, 0.001596],
            [161.030975, -129.313187, 0.001679],
            [184.758713, -199.424271, 0.001680],
        ]
        sampling_resolution = 1.0
        target_locations = xyz_to_locs(target_points, self.map)
        grp = GlobalRoutePlanner(self.map, sampling_resolution)
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
        driver_agent.agent.set_destination([destination.x, destination.y, destination.z])

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
                driver_agent.agent.set_destination([destination.x, destination.y, destination.z])

                # LOGGING INFO FOR DEBUGGING
                self.get_logger().info(f"Moving to waypoint {destination}, done_count: {current_waypoint_index}/{total_waypoints-1}")

            # DEBUG LOG DATA
            current_time = time.time()
            if current_time - self.last_debug_time >= self.debug_data_timeout:
                speed_kmh = get_speed(self.ego_vehicle)
                current_location = self.ego_vehicle.get_location()

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
