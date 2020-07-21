cmake_minimum_required(VERSION 2.8.3)
project(position_estimator)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  message_generation
  cv_bridge
  roscpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_ros
  object_detector
)
find_package( OpenCV REQUIRED )
find_package( PCL REQUIRED )

add_message_files(
    FILES
    LocalizedDetection.msg
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
   include
  CATKIN_DEPENDS 
   pcl_ros
   message_runtime 
   visualization_msgs
   sensor_msgs
   geometry_msgs
   roscpp
   object_detector
)
include_directories(
    include ${catkin_INCLUDE_DIRS}
    include ${OpenCV_INCLUDE_DIRS}
    include ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}_node.cpp src/PositionEstimator.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
