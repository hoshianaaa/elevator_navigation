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
  laser_geometry
  pcl_conversions
  pcl_msgs
  pcl_ros
  elevator_navigation_srv
  elevator_navigation_msgs
  roslib
  go_target
)

find_package(PkgConfig)
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)

if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
add_definitions(-DNEW_YAMLCPP)
endif()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES elevator_waypoints_nav
#  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs move_base move_base_msgs roscpp std_srvs tf
#  DEPENDS system_lib
)

include_directories(
	include
  	${catkin_INCLUDE_DIRS}
	${yaml-cpp_INCLUDE_DIRS}
)

add_library(action src/action.cpp)
add_library(panel_action src/panel_action.cpp)

add_executable(action_node src/action_node.cpp)
target_link_libraries(action_node
	${catkin_LIBRARIES}
	action
)

add_executable(panel_action_node src/panel_action_node.cpp)
target_link_libraries(panel_action_node
	${catkin_LIBRARIES}
	panel_action
	action
)

add_executable(push_bottun src/push_bottun.cpp)
add_dependencies(push_bottun ${elevator_navigation_srv_EXPORTED_TARGETS})
target_link_libraries(push_bottun
	${catkin_LIBRARIES}
	panel_action
)


add_executable(waypoints_nav src/waypoints_nav.cpp)
target_link_libraries(waypoints_nav
	${catkin_LIBRARIES}
	yaml-cpp
	action
)
