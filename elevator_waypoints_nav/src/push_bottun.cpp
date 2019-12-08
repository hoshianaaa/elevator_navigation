#include <ros/ros.h>
#include <elevator_waypoints_nav/panel_action.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "panel_action_node");
	ros::NodeHandle n;

	PanelAction pa;
  pa.straight(0.05);
  pa.straight(-0.05);
	return (0);
}
