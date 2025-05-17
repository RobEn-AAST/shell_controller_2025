#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import carla
import numpy as np
from ai_src.carla_others.agents.navigation.global_route_planner import GlobalRoutePlanner
from ai_src.carla_others.agents.navigation.basic_agent import BasicAgent
from ai_src.navigator.wp_utils import xyz_to_right_lane
from ai_src.navigator.tsp_solver import optimize_route_order
from ai_src.navigator.route_generator import generate_full_route

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

        client = carla.Client("localhost", 2000) # type: ignore
        client.set_timeout(20)
        world = client.get_world()
        carla_map = world.get_map()
        ego_vehicle = next(v for v in world.get_actors().filter("vehicle.*") if v.attributes.get("role_name") == "ego_vehicle")

        target_points = [
            [280.363739, 133.306351, 0.001746],
            [334.949799, 161.106171, 0.001736],
            [339.100037, 258.568939, 0.001679],
            [396.295319, 183.195740, 0.001678],
            [267.657074, 1.983160, 0.001678],
            [153.868896, 26.115866, 0.001678],
            [290.515564, 56.175072, 0.001677],
            [92.325722, 86.063644, 0.001677],
            [88.384346, 287.468567, 0.001728],
            [177.594101, 326.386902, 0.001677],
            [-1.646942, 197.501282, 0.001555],
            [59.701321, 1.970804, 0.001467],
            [122.100121, 55.142044, 0.001596],
            [161.030975, 129.313187, 0.001679],
            [184.758713, 199.424271, 0.001680],
        ]
        sampling_resolution = 1.0

        target_locations = xyz_to_right_lane(target_points, carla_map)

        grp = GlobalRoutePlanner(carla_map, sampling_resolution)

        optimized_targets = optimize_route_order(
            ego_vehicle.get_location(),
            target_locations,
            grp,
        )

        full_route = generate_full_route(ego_vehicle.get_location(), optimized_targets, grp)

        # ======= MOVE VEHICLE ========
        agent = BasicAgent(ego_vehicle)
        agent.ignore_traffic_lights(True)
        agent.set_target_speed(45)
        agent.set_global_plan(full_route)

        self.get_logger().info("Brain node started... about to move")
        while True:
            control = agent.run_step()  # Auto-generates throttle/brake/steering
            ego_vehicle.apply_control(control)


def main(args=None):
    rclpy.init(args=args)

    node = Brain()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
