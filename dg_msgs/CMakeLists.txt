cmake_minimum_required(VERSION 2.8.3)
project(dg_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  pluginlib
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  String.msg
)

generate_messages(DEPENDENCIES)
catkin_package(CATKIN_DEPENDS roscpp std_msgs message_runtime)




