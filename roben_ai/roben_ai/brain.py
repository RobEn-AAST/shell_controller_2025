#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import car_dreamer
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

        task, _ = car_dreamer.create_task("carla_custom")

        task.reset()
        while True:
            self.get_logger().info("stepped...")
            _, _, is_terminal, _ = task.step(12)
            if is_terminal:
                task.reset()
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)

    node = Brain()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
