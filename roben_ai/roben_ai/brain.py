import rclpy
from rclpy.node import Node
from ai_src import make_carla_env

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

        self.get_logger().info("Brain node started...")

        env = make_carla_env(eval=True)

        obs, _ = env.reset()
        while True:
            action = [2, 1]
            obs, r, terminated, truncated, _ = env.step(action)
            done = terminated or truncated
            print(f'reward: {r}')
            if done:
                # obs = env.reset()
                print('welp here went nothing')




def main(args=None):
    rclpy.init(args=args)

    node = Brain()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()