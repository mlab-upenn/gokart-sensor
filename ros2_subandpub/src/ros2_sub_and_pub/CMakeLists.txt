cmake_minimum_required(VERSION 3.5)
project(ros2_sub_and_pub)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system) 

add_executable(ros2_sub_and_pub src/main.cpp)
ament_target_dependencies(ros2_sub_and_pub rclcpp std_msgs sensor_msgs)

target_link_libraries(ros2_sub_and_pub
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
)

install(TARGETS
  ros2_sub_and_pub
  DESTINATION lib/${PROJECT_NAME})
  
ament_package()
