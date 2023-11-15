cmake_minimum_required(VERSION 3.0.2)
project(slam)

add_definitions(-std=c++14 -Wall -Wno-sign-compare -Wno-catch-value -Wno-switch -Wno-maybe-uninitialized -Werror=return-type)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

cs_add_library(
  ${PROJECT_NAME}
  src/slam.cpp
)

cs_add_executable(slam_ros_node src/slam_ros_node.cpp)
target_link_libraries(slam_ros_node ${PROJECT_NAME} ${PCL_LIBRARIES})

cs_install()
cs_export(LIBRARIES)