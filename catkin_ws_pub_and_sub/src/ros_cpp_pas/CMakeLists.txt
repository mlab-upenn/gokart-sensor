cmake_minimum_required(VERSION 3.0.2)
project(ros_cpp_pas)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(CATKIN_DEPENDS roscpp std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(ros_cpp_pas src/ros_cpp_pas.cpp)
target_link_libraries(ros_cpp_pas ${catkin_LIBRARIES})
