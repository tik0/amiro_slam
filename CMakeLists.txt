cmake_minimum_required(VERSION 2.8.3)
project(amiro_slam)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cmake_modules
  laser_geometry
  tf
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

LINK_DIRECTORIES (
)

add_executable(ros_laser_scan_to_ros_pointcloud2
  src/ros_laser_scan_to_ros_pointcloud2.cpp
)

target_link_libraries(ros_laser_scan_to_ros_pointcloud2
  ${catkin_LIBRARIES}
)

add_executable(dynamic_tf_with_odom
  src/dynamic_tf_with_odom.cpp
)

target_link_libraries(dynamic_tf_with_odom
  ${catkin_LIBRARIES}
)
