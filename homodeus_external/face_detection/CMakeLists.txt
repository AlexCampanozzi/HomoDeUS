cmake_minimum_required(VERSION 3.0.2)
project(face_detection)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pal_detection_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES face_detection
  CATKIN_DEPENDS roscpp rospy actionlib std_msgs actionlib_msgs message_runtime
#  DEPENDS system_lib
)

## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


