cmake_minimum_required(VERSION 2.8.3)
project(semantic_mapper)

find_package(catkin REQUIRED COMPONENTS
  rospy
  visualization_msgs
  position_estimator
)

catkin_package(
)
include_directories(
    include ${catkin_INCLUDE_DIRS}
)
