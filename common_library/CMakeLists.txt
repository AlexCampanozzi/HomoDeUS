cmake_minimum_required(VERSION 2.8.3)
project(common_library)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
catkin_python_setup()
catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)