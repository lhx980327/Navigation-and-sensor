cmake_minimum_required(VERSION 3.0.2)
project(gps_driver)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
)

## Declare ROS messages, services, and actions
add_message_files(
  FILES
  GPSInfo.msg
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy message_runtime
)

## Specify libraries to link executable targets against
#add_executable(gps_node src/gps_node.cpp)
#target_link_libraries(gps_node ${catkin_LIBRARIES} ...)

