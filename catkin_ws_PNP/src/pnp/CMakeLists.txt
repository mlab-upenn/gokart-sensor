cmake_minimum_required(VERSION 3.0.2)
project(pnp)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  rospy
  darknet_ros_msgs
  visualization_msgs
)
find_package(darknet_ros_msgs REQUIRED)

catkin_package(
  INCLUDE_DIRS scripts
  CATKIN_DEPENDS rospy darknet_ros_msgs visualization_msgs
) 

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/pnp.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
