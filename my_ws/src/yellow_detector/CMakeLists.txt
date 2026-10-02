cmake_minimum_required(VERSION 3.0.2)
project(yellow_detector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_generation
  rospy
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  YellowBalloonDetection.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    geometry_msgs
    message_runtime
    rospy
    sensor_msgs
    std_msgs
)

catkin_install_python(
  PROGRAMS scripts/yellow_detector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
