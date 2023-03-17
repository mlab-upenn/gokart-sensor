cmake_minimum_required(VERSION 3.0.2)
project(rosbag2npy)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  ros_numpy
)

catkin_package(
  INCLUDE_DIRS scripts
  CATKIN_DEPENDS rospy sensor_msgs ros_numpy
) 

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/rosbag2npy.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
