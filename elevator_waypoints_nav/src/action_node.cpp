#include <ros/ros.h>
#include <elevator_waypoints_nav/action.h>
#include <geometry_msgs/Point.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "action_node");

	geometry_msgs::Point start_point, goal_point, robot_point;
	start_point.x = 2;

	Action in_elevator(start_point, goal_point, true);
	Action out_elevator(goal_point, start_point);

	std::cout << "in" << std::endl;
	in_elevator.print_env_data();

	std::cout << "out" << std::endl;
	out_elevator.print_env_data();

	ros::Duration(3.0).sleep();

	in_elevator.get_robot_pose();
	out_elevator.get_robot_pose();

	in_elevator.print_robot_data();
	std::cout << "in" << std::endl;
	out_elevator.print_robot_data();
	std::cout << "out" << std::endl;

	for(int i=0;i<360;i++){	
		start_point.x = 0;
		start_point.y = 0;
		goal_point.x = 2;
		goal_point.y = 2;
		robot_point.x = std::cos(i/360.0*2*M_PI);
		robot_point.y = std::sin(i/360.0*2*M_PI);

		double ang = in_elevator.calc_goal_angle(start_point, robot_point, true);
		std::cout << "ang:" << i <<  " :" << ang <<  std::endl;
	}

	ros::spin();
	return (0);
}
