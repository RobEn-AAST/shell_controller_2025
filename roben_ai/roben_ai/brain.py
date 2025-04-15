import rclpy
from rclpy.node import Node
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber

from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image, PointCloud2, NavSatFix, Imu
from carla_msgs.msg import CarlaEgoVehicleStatus, CarlaLaneInvasionEvent, CarlaCollisionEvent

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs_py import point_cloud2

import cv2
import matplotlib.pyplot as plt

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

        rgb_sub = Subscriber(self, Image, "/carla/ego_vehicle/rgb_front/image")
        lidar_sub = Subscriber(self, PointCloud2, "/carla/ego_vehicle/vlp16_1")
        gps_sub = Subscriber(self, NavSatFix, "/carla/ego_vehicle/gnss")
        status_sub = Subscriber(self, CarlaEgoVehicleStatus, "/carla/ego_vehicle/vehicle_status")
        imu_sub = Subscriber(self, Imu, "/carla/ego_vehicle/imu")
        

        # Sync messages within 0.1s window
        ats = ApproximateTimeSynchronizer(
            [rgb_sub, lidar_sub, gps_sub, status_sub, imu_sub],
            queue_size=30,
            slop=0.1,  # 100ms tolerance
        )
        ats.registerCallback(self.sync_sensors_callback)
        self.cv_bridge = CvBridge()

        # headerless sensors
        self.speed = np.array([0])
        self.create_subscription(
            Float32, "/carla/ego_vehicle/speedometer", self.speed_cb, 10
        )

        self.create_subscription(
            String, "/carla/map", self.map_cb, 10
        )
        
        # event driven
        self.lane_invaded = np.array([0])
        self.create_subscription(CarlaLaneInvasionEvent, '/carla/ego_vehicle/lane_invasion', self.lane_invaded_cb, 10)

        self.collision_detected = np.array([0])
        self.create_subscription(CarlaLaneInvasionEvent, '/carla/ego_vehicle/collision', self.collision_detected_cb, 10)

        self.get_logger().info("brain started...")

    def speed_cb(self, speed_msg: Float32):
        self.speed = np.array([speed_msg.data], dtype=np.float32)

    def map_cb(self, map_msg: String):
        print('test')

    def lane_invaded_cb(self, lane_invaded_msg: CarlaLaneInvasionEvent):
        # Check if the vehicle has invaded a lane
        print('what the hell did we just get?')
        self.lane_invaded = np.array(lane_invaded_msg.crossed_lane_markings)

    def collision_detected_cb(self, collision_msg: CarlaCollisionEvent):
        print('what the hell did we just get?')
        self.collision_detected = np.array([collision_msg.normal_impulse.x, collision_msg.normal_impulse.y, collision_msg.normal_impulse.z])



    def sync_sensors_callback(
        self,
        rgb_msg: Image,
        lidar_msg: PointCloud2,
        gps_msg: NavSatFix,
        status_msg: CarlaEgoVehicleStatus,
        imu_msg: Imu,
    ):
        # parse images
        try:
            rgb_img = self.cv_bridge.imgmsg_to_cv2(
                rgb_msg, desired_encoding="passthrough"
            )
            rgb_img = np.array(rgb_img)[:, :, :3]  # HxWxC

            # cv2.imshow("img_rgb", rgb_img)
            # cv2.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image {e}")

        # parse lidar
        lidar_pnts = np.array(
            list(point_cloud2.read_points(lidar_msg, field_names=["x", "y", "z"]))
        )
        lidar_pnts = np.column_stack(
            (lidar_pnts["x"], lidar_pnts["y"], lidar_pnts["z"])
        )

        # plt.scatter(
        #     lidar_pnts[:, 0], lidar_pnts[:, 1], s=1, c=lidar_pnts[:, 2], cmap="viridis"
        # )
        # plt.xlabel("X (forward)")
        # plt.ylabel("Y (left-right)")
        # plt.colorbar(label="Height (Z)")
        # plt.show()

        # parse gps
        gps_cords = np.array([gps_msg.latitude, gps_msg.longitude], dtype=np.float32)

def main(args=None):
    rclpy.init(args=args)

    node = Brain()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
