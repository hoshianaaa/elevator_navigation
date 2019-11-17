#include <ros/ros.h>
#include <elevator_waypoints_nav/panel_action.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "panel_action_node");

	PanelAction action;
	action.run();

	ros::spin();
	return (0);
}
