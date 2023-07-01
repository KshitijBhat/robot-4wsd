cmake_minimum_required(VERSION 3.0.2)
project(four_wheel_steering_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  controller_interface
  four_wheel_steering_msgs
  hardware_interface
  nav_msgs
  pluginlib
  realtime_tools
  roscpp
  tf
  urdf_geometry_parser
)

find_package(Boost REQUIRED)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    controller_interface
    four_wheel_steering_msgs
    hardware_interface
    nav_msgs
    realtime_tools
    roscpp
    tf
  DEPENDS Boost
)


###########
## Build ##
###########

# Specify header include paths
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/four_wheel_steering_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install plugins
install(FILES four_wheel_steering_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
