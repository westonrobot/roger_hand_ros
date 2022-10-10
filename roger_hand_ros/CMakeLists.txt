cmake_minimum_required(VERSION 3.0.2)
project(roger_hand_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roger_hand_msgs
  roscpp
  rospy
)

catkin_package(
 CATKIN_DEPENDS roger_hand_msgs roscpp rospy
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(hand_control
  include/roger_hand_ros/hand_control.h
  src/hand_control.cpp
)
add_dependencies(hand_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hand_control
  ${catkin_LIBRARIES}
)


add_executable(hand_driver_node
src/hand_driver_node.cpp
src/hand_driver.cpp
include/roger_hand_ros/hand_driver.h
)
add_dependencies(hand_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hand_driver_node
  ${catkin_LIBRARIES}
  hand_control
)