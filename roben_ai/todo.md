first get the image and any sensor data available
use the sensors and image to guide the car
use the image later to tell you the speed limit you should be walking with

We coudl separate the backbone into its own node, and both the rl algoirhtm and the speed sensor detector would subscribe to it and start doing there own things



look at the following sensors
  ! indicates integrated and will be used 100%
  * indicates integrated already but not sure if will be used or not
  x indicates looked at but won't be integrated
  -> indicates looked at but integration in progress

  !! /carla/ego_vehicle/rgb_front/image          - Front camera (RGB)
  * /carla/ego_vehicle/depth_middle/image       - Depth camera
  ! /carla/ego_vehicle/vlp16_1                  - LiDAR (PointCloud2)
  !! /carla/ego_vehicle/gnss                     - GPS coordinates
  * /carla/ego_vehicle/imu                      - Acceleration/Orientation
  /carla/ego_vehicle/odometry                 - Position/Velocity
  /carla/ego_vehicle/collision                - Collision detection
  /carla/ego_vehicle/lane_invasion            - Lane departure
  !! /carla/ego_vehicle/speedometer              - Current speed
  * /carla/ego_vehicle/vehicle_status           - Gear, control status, etc. (will neglect control status from here tho)