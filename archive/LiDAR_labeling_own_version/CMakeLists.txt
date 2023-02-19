cmake_minimum_required(VERSION 3.0.2)
project(lidar_labelling)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  visualization_msgs
  ros_numpy
)

catkin_package(
  INCLUDE_DIRS scripts
  CATKIN_DEPENDS rospy std_msgs sensor_msgs visualization_msgs ros_numpy
) 

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/lidar_labelling.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
