cmake_minimum_required(VERSION 2.8.3)
project(gps_tools)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2_ros
  tf
  dji_sdk
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  gps_tools_srvs
)

catkin_python_setup()
