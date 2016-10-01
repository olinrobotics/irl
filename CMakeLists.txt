cmake_minimum_required(VERSION 2.8.3)
project(edwin)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	roslib
	std_msgs
	message_generation
##	rosserial_arduino
##	rosserial_client
	cv_bridge
)

## Generate messages in the 'msg' folder
add_message_files(
	DIRECTORY
	msg
	FILES
	Edwin_Shape.msg

)

##Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
	CATKIN_DEPENDS message_runtime std_msgs)






include_directories(
  ${catkin_INCLUDE_DIRS}
)
