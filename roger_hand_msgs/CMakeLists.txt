cmake_minimum_required(VERSION 3.0.2)
project(roger_hand_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(FILES
  hand_state.msg
)

add_service_files(FILES
  finger_pose.srv
  clear_error.srv
  ampere_feedback.srv
  hand_enable.srv
  hand_pose.srv
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)