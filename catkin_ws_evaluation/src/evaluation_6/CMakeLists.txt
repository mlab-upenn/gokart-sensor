cmake_minimum_required(VERSION 3.0.2)
project(evaluation_6)

set(CMAKE_CXX_STANDARD 14)

set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs message_filters visualization_msgs)
find_package(darknet_ros_msgs REQUIRED)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_filters darknet_ros_msgs visualization_msgs
)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_install_python(PROGRAMS src/evaluation_6.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
