cmake_minimum_required(VERSION 3.5)
project(data_aug_ros2)

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
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system) 
include_directories( include ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} /usr/local/include/pcl-1.10)

add_executable(imu_listener src/imu_listener.cpp)
ament_target_dependencies(imu_listener rclcpp std_msgs sensor_msgs)
add_executable(gnss_filter src/gnss_filter.cpp)
ament_target_dependencies(gnss_filter rclcpp std_msgs sensor_msgs)
add_executable(lidar_filter src/lidar_filter.cpp)
ament_target_dependencies(lidar_filter rclcpp std_msgs sensor_msgs pcl_conversions PCL)
ament_export_include_directories(include)
target_link_libraries(lidar_filter
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
  ${PCL_LIBRARIES}
)

install(TARGETS
  imu_listener
  gnss_filter
  lidar_filter
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
