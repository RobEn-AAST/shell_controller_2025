cmake_minimum_required(VERSION 3.0.2)
project(roben_ai)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  carla_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs carla_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Set up Python package
catkin_python_setup()

#############
## Install ##
#############

# Install Python module and requirements installer
catkin_install_python(PROGRAMS
  scripts/install_requirements.py
  roben_ai/brain.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Rename brain.py to brain for cleaner execution
install(
  FILES ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/brain.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  RENAME brain
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

# Install directory structure
install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
)

# Create a requirements.txt file based on setup.py dependencies
install(FILES
  requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Custom ##
#############

# Add dependency installation as a build step
add_custom_target(install_requirements ALL
  COMMAND ${PYTHON_EXECUTABLE} ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/install_requirements.py
  DEPENDS ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/install_requirements.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Installing Python requirements for roben_ai"
)

# Make brain the main target - similar to shell_simulation_node
add_custom_target(brain ALL
  DEPENDS install_requirements ${CMAKE_CURRENT_SOURCE_DIR}/roben_ai/brain.py
  COMMENT "Building brain node target"
)