cmake_minimum_required(VERSION 2.8.3)
project(semantic_mapper)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  position_estimator
)
find_package( OpenCV REQUIRED )
#find_package( PCL REQUIRED )

catkin_package(
  INCLUDE_DIRS
   include
  CATKIN_DEPENDS 
   message_runtime 
   visualization_msgs
   roscpp
   position_estimator
)
include_directories(
    include ${catkin_INCLUDE_DIRS}
    # include ${OpenCV_INCLUDE_DIRS}
    # include ${PCL_INCLUDE_DIRS}
)

#link_directories(${PCL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}_node.cpp src/SemanticMapper.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
