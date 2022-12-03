cmake_minimum_required(VERSION 3.0.2)
project(pcl2velodyne)


find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  rospy
  std_msgs
  sensor_msgs
  ros_numpy
)

catkin_package(CATKIN_DEPENDS pcl_ros rospy std_msgs sensor_msgs ros_numpy)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
