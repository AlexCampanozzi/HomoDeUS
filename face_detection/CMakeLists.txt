cmake_minimum_required(VERSION 2.8.3)
project(face_detection)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport    
  pal_detection_msgs
  sensor_msgs
  roscpp
)
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS sensor_msgs pal_detection_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

###########
## Build ##
###########

## Declare a cpp executable

 add_executable(${PROJECT_NAME}_node
   src/face_detector.cpp
   src/face_detection_node.cpp
 )
###add_dependencies(${PROJECT_NAME} pal_detection_msgs_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


#############
## Install ##
#############

#install(TARGETS ${PROJECT_NAME}_node
#       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(DIRECTORY launch config 
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#        USE_SOURCE_PERMISSIONS)
