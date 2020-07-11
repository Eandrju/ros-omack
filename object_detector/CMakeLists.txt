cmake_minimum_required(VERSION 2.8.3)
project(object_detector)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)

add_message_files(
  FILES
  RGB.msg
  Detection.msg
  DetectionBundle.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
