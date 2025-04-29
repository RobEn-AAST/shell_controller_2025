cmake_minimum_required(VERSION 3.8)
project(roben_ai)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# Use carla_msgs if available in ROS2, otherwise comment out
find_package(carla_msgs REQUIRED)

###########
## Build ##
###########

# Include directories for C++
include_directories(include)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

#############
## Install ##
#############

# Install Python scripts with proper execution permissions
install(PROGRAMS
  scripts/install_requirements.py
  ${PROJECT_NAME}/brain.py
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

# Rename brain.py to brain for cleaner execution
install(FILES
  ${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/brain.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME brain
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

install(FILES
  requirements.txt
  DESTINATION share/${PROJECT_NAME}
)

#############
## Custom ##
#############

# Add dependency installation as a build step
add_custom_target(install_requirements ALL
  COMMAND ${PYTHON_EXECUTABLE} ${PROJECT_SOURCE_DIR}/scripts/install_requirements.py
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  COMMENT "Installing Python requirements for roben_ai"
)

# Make brain the main target
add_custom_target(brain ALL
  DEPENDS install_requirements ${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/brain.py
  COMMENT "Building brain node target"
)

# Export package dependencies
ament_export_dependencies(
  rclcpp
  rclpy
  std_msgs
  sensor_msgs
  geometry_msgs
  carla_msgs
)

ament_package()