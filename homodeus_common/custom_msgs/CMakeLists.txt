cmake_minimum_required(VERSION 3.0.2)
project(custom_msgs)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  actionlib
  actionlib_msgs
  std_msgs
  move_base_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  GoToResult.msg
  FacePosition.msg
  FacePositions.msg
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
  ttsAction.action
  scenario_manager.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES base_navigation
  CATKIN_DEPENDS geometry_msgs roscpp rospy actionlib std_msgs move_base_msgs actionlib_msgs message_runtime
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

