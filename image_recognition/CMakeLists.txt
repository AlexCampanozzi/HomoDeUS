cmake_minimum_required(VERSION 2.8.3)
project(image_recognition)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport    
  pal_detection_msgs
  sensor_msgs
  roscpp
)
find_package(OpenCV REQUIRED)

##########
# Flags ##
##########

#add_definitions(-DPAL_MODULE_NAME=\"VISION\")
#add_definitions(-DPAL_PROJECT_NAME=\"${PROJECT_NAME}\")
#add_definitions(-DPAL_LIBRARY_NAME=\"${PROJECT_NAME}\")

###################################
## catkin specific configuration ##
###################################


catkin_package(
  CATKIN_DEPENDS sensor_msgs pal_detection_msgs
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

###########
## Build ##
###########

## Declare a cpp executable
add_executable(image_recognition src/face_detector.cpp)
add_dependencies(image_recognition pal_detection_msgs_generate_messages_cpp)
target_link_libraries(image_recognition ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


#############
## Install ##
#############

install(TARGETS image_recognition
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config 
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)
