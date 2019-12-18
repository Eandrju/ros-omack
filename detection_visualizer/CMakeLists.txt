cmake_minimum_required(VERSION 2.8.3)
project(detection_visualizer)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  object_detector
  rospy
  sensor_msgs
  cv_bridge
)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
