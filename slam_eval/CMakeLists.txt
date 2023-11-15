cmake_minimum_required(VERSION 3.0.2)
project(slam_eval)

find_package(catkin REQUIRED COMPONENTS
  rospy
  cv_bridge
  message_generation
  geometry_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)