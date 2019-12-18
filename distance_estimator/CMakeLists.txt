cmake_minimum_required(VERSION 2.8.3)
project(distance_estimator)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  cv_bridge
  roscpp
  sensor_msgs
  visualization_msgs
  pcl_ros
  object_detector
)
find_package( OpenCV REQUIRED )
#find_package( PCL REQUIRED )

catkin_package(
  INCLUDE_DIRS
   include
  CATKIN_DEPENDS 
   pcl_ros
   message_runtime 
   visualization_msgs
   sensor_msgs
   roscpp
   object_detector
)
include_directories(
    include ${catkin_INCLUDE_DIRS}
    include ${OpenCV_INCLUDE_DIRS}
    include ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}_node.cpp src/DistanceEstimator.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
