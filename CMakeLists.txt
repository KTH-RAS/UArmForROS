cmake_minimum_required(VERSION 2.8.3)
project(uarm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  visualization_msgs
  interactive_markers
)
add_message_files(
  FILES
	Angles.msg
	Coords.msg
	CoordsWithTime.msg
	CoordsWithTS4.msg
)

add_service_files(
  FILES
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES uarm
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(uarm_status src/uarm_status.cpp)
target_link_libraries(uarm_status ${catkin_LIBRARIES})
