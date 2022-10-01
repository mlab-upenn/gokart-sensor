cmake_minimum_required(VERSION 3.23)
project(SensorPipeline)

set(CMAKE_CXX_STANDARD 14)

# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)
set(OpenCV_DIR /usr/share/OpenCV4/)

find_package(catkin REQUIRED COMPONENTS cv_bridge roscpp sensor_msgs)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(SensorPipeline main.cpp)

target_link_libraries(SensorPipeline ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

