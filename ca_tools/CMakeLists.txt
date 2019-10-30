cmake_minimum_required(VERSION 2.8.3)
project(ca_tools)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs sensor_msgs)

include_directories(${OpenCV_INCLUDE_DIRS})

add_action_files(
  DIRECTORY action
  )
 
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    joy_teleop
    geometry_msgs
    rospy
    actionlib_msgs
    std_msgs
    cv_bridge
    sensor_msgs
)


install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
