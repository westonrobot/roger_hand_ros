cmake_minimum_required(VERSION 2.8.3)
project(roger_hand_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)


add_service_files(FILES
  finger_pose.srv
  clear_error.srv
  ampere_feedback.srv
  hand_enable.srv
  hand_pose.srv
)



add_message_files(FILES
  hand_state.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES roger_hand_ros
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
# ${catkin_INCLUDE_DIRS}
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

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)