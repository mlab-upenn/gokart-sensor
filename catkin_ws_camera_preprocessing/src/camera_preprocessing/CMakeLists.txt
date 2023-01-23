cmake_minimum_required(VERSION 3.0.2)
project(camera_preprocessing)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  serial
)

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES imu_sensor_msgs
  CATKIN_DEPENDS rospy std_msgs sensor_msgs std_msgs serial
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/camera_preprocessing.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
