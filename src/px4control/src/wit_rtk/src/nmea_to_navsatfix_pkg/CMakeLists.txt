cmake_minimum_required(VERSION 3.0.2)
project(nmea_to_navsatfix_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

install(PROGRAMS
  scripts/nmea_to_navsatfix.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
