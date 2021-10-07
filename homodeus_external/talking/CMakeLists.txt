cmake_minimum_required(VERSION 2.8.3)
project(talking_node)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)
## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
  ttsAction.action
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
  CATKIN_DEPENDS geometry_msgs roscpp rospy actionlib std_msgs actionlib_msgs message_runtime
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
