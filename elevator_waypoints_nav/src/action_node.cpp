#include <ros/ros.h>
#include <elevator_waypoints_nav/action.h>
#include <geometry_msgs/Point.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "action_node");
	geometry_msgs::Point start_point, goal_point;
	start_point.x = 2;

	Action in_elevator(start_point, goal_point, true);
	Action out_elevator(goal_point, start_point);

	std::cout << "in" << std::endl;
	in_elevator.print_env_data();

	std::cout << "out" << std::endl;
	out_elevator.print_env_data();

	in_elevator.get_robot_pose();
	out_elevator.get_robot_pose();

	in_elevator.print_robot_data();
	std::cout << "in" << std::endl;
	out_elevator.print_robot_data();
	std::cout << "out" << std::endl;

	ros::spin();
	return (0);
}
