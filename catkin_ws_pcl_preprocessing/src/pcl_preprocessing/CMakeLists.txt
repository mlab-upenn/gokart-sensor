cmake_minimum_required(VERSION 3.0.2)
project(pcl_preprocessing)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  ros_numpy
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS pcl_ros roscpp std_msgs sensor_msgs ros_numpy)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(pcl_preprocessing src/pcl_preprocessing.cpp)
target_link_libraries(pcl_preprocessing ${catkin_LIBRARIES})
