cmake_minimum_required(VERSION 3.0.2)
project(can_msg_parser)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  apa_msgs
  can_msgs
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  ros_esdcan_bridge
  tf
)

find_package(Eigen3 REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES can_msg_parser
#  CATKIN_DEPENDS can_msgs geometry_msgs lidar_perception nav_msgs ros_esdcan_bridge roscpp sensor_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(rt3000_msg_parser src/rt3000_msg_parser.cpp)
target_link_libraries(rt3000_msg_parser ${ros_esdcan_bridge_LIBRARIES} ${catkin_LIBRARIES})

add_executable(chasis_msg_parser src/chasis_msg_parser.cpp)
target_link_libraries(chasis_msg_parser ${ros_esdcan_bridge_LIBRARIES} ${catkin_LIBRARIES})

add_executable(parking_info_parser src/parking_info_parser.cpp)
target_link_libraries(parking_info_parser ${ros_esdcan_bridge_LIBRARIES} ${catkin_LIBRARIES})

add_executable(kvaser_powerUp src/kvaser_powerUp.cpp)
target_link_libraries(kvaser_powerUp ${catkin_LIBRARIES})

add_executable(powerUp src/powerUp.cpp)
target_link_libraries(powerUp ${catkin_LIBRARIES})