cmake_minimum_required(VERSION 3.1)
project(sd_autoware)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  autoware_msgs
  sd_control_msgs
)

catkin_package()

catkin_install_python(PROGRAMS nodes/sd_autoware.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
