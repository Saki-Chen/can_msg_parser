cmake_minimum_required(VERSION 3.0.2)
project(can_msg_parser)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

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
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS 
    apa_msgs
    can_msgs
    geometry_msgs
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    ros_esdcan_bridge
    tf
  DEPENDS Eigen3
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(rt3000_msg_parser src/rt3000_msg_parser.cpp)
target_link_libraries(rt3000_msg_parser ${ros_esdcan_bridge_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(rt3000_msg_parser ${catkin_EXPORTED_TARGETS})

add_executable(chasis_msg_parser src/chasis_msg_parser.cpp)
target_link_libraries(chasis_msg_parser ${ros_esdcan_bridge_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(chasis_msg_parser ${catkin_EXPORTED_TARGETS})

add_executable(parking_info_parser src/parking_info_parser.cpp)
target_link_libraries(parking_info_parser ${ros_esdcan_bridge_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(parking_info_parser ${catkin_EXPORTED_TARGETS})
