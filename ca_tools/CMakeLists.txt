cmake_minimum_required(VERSION 2.8.3)
project(ca_tools)

find_package(catkin REQUIRED
  actionlib_msgs
  message_generation
  geometry_msgs
  )

add_action_files(
  FILES
  FollowLine.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    joy_teleop
    geometry_msgs
    rospy
    actionlib_msgs
    message_runtime
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
