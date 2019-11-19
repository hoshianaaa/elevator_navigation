#include <ros/ros.h>
#include <elevator_waypoints_nav/panel_action.h>
#include <elevator_waypoints_nav/search_panel.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "panel_action_node");
	
	SearchPanel sp;
	sp.run(1);
	sp.run(0);

	return (0);
}
