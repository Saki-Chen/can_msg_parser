cmake_minimum_required(VERSION 3.0.2)
project(can_msg_parser)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

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