cmake_minimum_required(VERSION 2.8.3)
project(face_detection)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport    
  pal_detection_msgs
  sensor_msgs
  roscpp
  message_generation
)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  FacePosition.msg
  FacePositions.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
###################################
## catkin specific configuration ##
###################################


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS sensor_msgs pal_detection_msgs message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

###########
## Build ##
###########

## Declare a cpp executable

 add_executable(${PROJECT_NAME}_node
   src/homodeus_proc_face_detection.cpp
 )
###
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

