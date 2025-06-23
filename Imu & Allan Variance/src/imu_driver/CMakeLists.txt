cmake_minimum_required(VERSION 3.0.2)
project(imu_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  message_generation
)


add_message_files(
  FILES
  imu_msg.msg
  
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

catkin_install_python(
  PROGRAMS python/driver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )