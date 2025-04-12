import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs_py import point_cloud2

import cv2
import matplotlib.pyplot as plt


class Brain(Node):
    def __init__(self):
        """
        The main controller of them all, the ultimate god of this body!
        Takes the image, and any other sensors we could find,
        sends them to an actor critic policy, and retreives the action we are asked to execute
        And executes it! voalla!!
        """
        super().__init__("brain")

        # sensor topics
        rgb_camera_topic = "/carla/ego_vehicle/rgb_front/image"
        depth_camera_topic = "/carla/ego_vehicle/depth_middle/image"
        lidar_topic = "/carla/ego_vehicle/vlp16_1"

        self.rgb_cam_subscriber_ = self.create_subscription(
            Image,
            rgb_camera_topic,
            self.rgb_cam_cb,
            10,
        )

        self.depth_cam_subscriber_ = self.create_subscription(
            Image,
            depth_camera_topic,
            self.depth_cam_cb,
            10,
        )

        self.lidar_subscriber_ = self.create_subscription(
            PointCloud2,
            lidar_topic,
            self.lidar_cb,
            10,
        )

        self.cv_bridge_ = CvBridge()

        self.get_logger().info("brain started...")

    def rgb_cam_cb(self, img_msg: Image):
        try:
            cv_image = self.cv_bridge_.imgmsg_to_cv2(
                img_msg, desired_encoding="passthrough"
            )
            img_np = np.array(cv_image)[:, :, :3]  # HxWxC

            cv2.imshow("img_rgb", img_np)
            cv2.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image {e}")

    def depth_cam_cb(self, img_msg: Image):
        try:
            cv_image = self.cv_bridge_.imgmsg_to_cv2(
                img_msg, desired_encoding="passthrough"
            )
            img_np = np.array(cv_image)  # HxWxC

            cv2.imshow("img_depth", img_np)
            cv2.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image {e}")

    def lidar_cb(self, msg: PointCloud2):
        points = np.array(
            list(point_cloud2.read_points(msg, field_names=["x", "y", "z"]))
        )
        points = np.column_stack((points["x"], points["y"], points["z"]))

        # Create top-down view (bird's eye)
        plt.scatter(points[:, 0], points[:, 1], s=1, c=points[:, 2], cmap="viridis")
        plt.xlabel("X (forward)")
        plt.ylabel("Y (left-right)")
        plt.colorbar(label="Height (Z)")
        plt.show()


def main(args=None):
    rclpy.init(args=args)

    node = Brain()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
