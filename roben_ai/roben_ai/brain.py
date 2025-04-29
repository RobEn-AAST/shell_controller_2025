#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ai_src import make_carla_env

class Brain(Node):
    def __init__(self):
        super().__init__('brain')
        self.get_logger().info("Brain node started...")
        
        # Initialize CARLA Gym environment
        self.env = make_carla_env(eval=True)
        self.obs, _ = self.env.reset()
        self.get_logger().info("done resetting...")
        
        # Create a timer to run the control loop
        self.timer = self.create_timer(0.1, self.run_step)  # 10 Hz
        
    def run_step(self):
        action = [2, 1]  # Replace with your RL policy later
        obs, reward, terminated, truncated, _ = self.env.step(action)
        self.get_logger().info(f"Reward: {reward}")
        
        # Check if episode is done
        if terminated or truncated:
            self.get_logger().info("Episode finished, resetting environment...")
            self.obs, _ = self.env.reset()

def main(args=None):
    rclpy.init(args=args)
    brain_node = Brain()
    rclpy.spin(brain_node)
    brain_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()