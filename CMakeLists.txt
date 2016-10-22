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
	image_transport
	geometry_msgs
	tf
)

# Find OpenNI
find_package(PkgConfig)
pkg_check_modules(OpenNI REQUIRED libopenni)

#Find OpenGL
find_package(OpenGL REQUIRED)

#Find GLUT
find_package(GLUT REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
	DIRECTORY
	msg
	FILES
	Edwin_Shape.msg
	pointerpos.msg
	edwin_gestures.msg

)

##Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)


catkin_package(
	CATKIN_DEPENDS message_runtime std_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenNI_INCLUDEDIR}
  ${OpenCV_INCLUDE_DIRS}
  ${OpenGL_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
)

include_directories("tools/nite/Include")
include_directories("tools/nite/Source/XnCommon")

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${GLUT_LIBRARY_DIRS}
		 ${OpenGL_LIBRARY_DIRS})

add_executable(edwin_wave src/SingleControl/main.cpp)
add_dependencies(edwin_wave geometry_msgs_gencpp ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(edwin_wave   ${catkin_LIBRARIES}
				   ${OpenNI_LIBRARIES}
				   ${OpenGL_INCLUDE_DIRS} -lXnVNite_1_5_2
				   ${GLUT_LIBRARY}
)



include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenNI_INCLUDEDIR}
  ${OpenCV_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
  ${OpenGL_INCLUDE_DIRS}
)

include_directories("tools/nite/Include")
include_directories("tools/nite/Source/XnCommon")

add_executable(edwin_bodies 
		src/SceneAnalysis/main.cpp
		src/SceneAnalysis/SceneDrawer.cpp
		src/SceneAnalysis/signal_catch.cpp
)

add_dependencies(edwin_bodies geometry_msgs_gencpp ${PROJECT_NAME}_generate_messages_cpp)

#MESSAGE( STATUS "OPENNI LIBRARIES: " ${OpenNI_LIBRARIES} )

target_link_libraries(edwin_bodies ${OpenNI_LIBRARIES} -lXnVNite_1_5_2
				   ${GLUT_LIBRARY} -lglut
				   ${OpenGL_LIBRARIES} -lGL
				   ${catkin_LIBRARIES})

install(TARGETS edwin_wave
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS edwin_bodies
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
	PATTERN ".svn" EXCLUDE)

install(FILES package.xml 
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
