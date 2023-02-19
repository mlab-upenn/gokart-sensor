cmake_minimum_required(VERSION 3.0.2)
project(pnp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
)
find_package(darknet_ros_msgs REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp visualization_msgs darknet_ros_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(pnp src/pnp.cpp)
target_link_libraries(pnp ${catkin_LIBRARIES})
