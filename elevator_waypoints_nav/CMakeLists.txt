cmake_minimum_required(VERSION 2.8.3)
project(elevator_waypoints_nav)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  move_base
  move_base_msgs
  roscpp
  std_srvs
  tf
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES elevator_waypoints_nav
#  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs move_base move_base_msgs roscpp std_srvs tf
#  DEPENDS system_lib
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

add_library(action src/action.cpp)

add_executable(action_node src/action_node.cpp)
target_link_libraries(action_node
	${catkin_LIBRARIES}
	action
)
