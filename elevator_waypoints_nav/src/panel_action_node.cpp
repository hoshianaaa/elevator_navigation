#include <ros/ros.h>
#include <elevator_waypoints_nav/panel_action.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "panel_action_node");
	
	PanelAction pa;
	pa.rotate(M_PI/7);
	pa.go_panel(0.3);

	return (0);
}
