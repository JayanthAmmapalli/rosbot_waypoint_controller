cmake_minimum_required(VERSION 3.0.2)
project(waypoint_controller_pd)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
  tf
)

add_service_files(
  FILES
  Waypoint.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs message_runtime tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

