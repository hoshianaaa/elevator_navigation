#include <ros/ros.h>
#include <elevator_waypoints_nav/action.h>
#include <geometry_msgs/Point.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "action_node");

	geometry_msgs::Point start_point, goal_point, robot_point;
	goal_point.x = -6;
	goal_point.y = -1;

	Action in_elevator(start_point, goal_point, true);
	//Action out_elevator(goal_point, start_point, false);

	ros::Duration(5.0).sleep();
	
	in_elevator.move();
	//out_elevator.move();

	ros::spin();
	return (0);
}
