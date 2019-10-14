#include <ros/ros.h>
#include <elevator_waypoints_nav/action.h>
#include <geometry_msgs/Point.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "action_node");
	geometry_msgs::Point start_point, goal_point;
	Action ac(start_point, goal_point);
	ac.move();
	ros::spin();
	return (0);
}
