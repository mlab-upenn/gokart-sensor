cmake_minimum_required(VERSION 3.23)
project(gnss_driver)

set(CMAKE_CXX_STANDARD 14)



find_package(catkin REQUIRED COMPONENTS serial rospy gps_common std_msgs message_generation)

add_message_files(FILES lat_long_heading.msg)

generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)


include_directories(include ${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS gnss_node.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#target_link_libraries(GNSSDriver ${catkin_LIBRARIES})
