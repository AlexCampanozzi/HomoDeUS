cmake_minimum_required(VERSION 3.0.2)
project(base_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  actionlib
  actionlib_msgs
  std_msgs
  move_base_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES base_navigation
  CATKIN_DEPENDS geometry_msgs roscpp rospy actionlib std_msgs move_base_msgs actionlib_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 catkin_install_python(PROGRAMS
   scripts/navigator.py
   scripts/navigator_tester.py
   scripts/goal_setter.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
 install(FILES
   launch/navtest.launch
   launch/bhvr_nav_listener.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
