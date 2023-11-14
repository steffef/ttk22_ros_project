cmake_minimum_required(VERSION 3.0.2)
project(slam_eval)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  message_generation
  geometry_msgs
)

catkin_python_setup()

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp cv_bridge tf message_runtime geometry_msgs visualization_msgs pcl_ros
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/slam_eval.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)