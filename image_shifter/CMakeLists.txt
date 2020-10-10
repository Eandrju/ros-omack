cmake_minimum_required(VERSION 2.8.3)
project(image_shifter)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
)


catkin_package(
    # CATKIN_DEPENDS message_runtime
)

# include_directories(
#   ${catkin_INCLUDE_DIRS}
# )
