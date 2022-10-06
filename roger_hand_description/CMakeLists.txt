cmake_minimum_required(VERSION 3.0.2)
project(roger_hand_description)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  ros_controllers
  controller_manager
  joint_state_publisher
  joint_state_publisher_gui
  robot_state_publisher
  roscpp
  rospy
  rviz
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)