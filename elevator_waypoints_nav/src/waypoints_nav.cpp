#include <elevator_waypoints_nav/waypoints_nav.h>

WaypointsNavigation::WaypointsNavigation() :
	move_base_action_("move_base", true),
	rate_(10)
{
	while((move_base_action_.waitForServer(ros::Duration(1.0)) == false) && (ros::ok() == true))
	{
		ROS_INFO("waiting...");
	}

	world_frame_ = "map";
}

void WaypointsNavigation::startNavigationGL(const geometry_msgs::Point &dest){
	geometry_msgs::Pose pose;
	pose.position = dest;
	pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
	startNavigationGL(pose);
}

void WaypointsNavigation::startNavigationGL(const geometry_msgs::Pose &dest){
	move_base_msgs::MoveBaseGoal move_base_goal;
	move_base_goal.target_pose.header.stamp = ros::Time::now();
	move_base_goal.target_pose.header.frame_id = world_frame_;
	move_base_goal.target_pose.pose.position = dest.position;
	move_base_goal.target_pose.pose.orientation = dest.orientation;
	move_base_action_.sendGoal(move_base_goal);
	std::cout << "send goal" << std::endl;
}

void WaypointsNavigation::sleep(){
	rate_.sleep();
	ros::spinOnce();
}

void WaypointsNavigation::run(){
	while(ros::ok()){
		sleep();
	}
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, ROS_PACKAGE_NAME);
	WaypointsNavigation w_nav;

	geometry_msgs::Point p;
	p.x = -3;
	w_nav.startNavigationGL(p);
	w_nav.sleep();
	w_nav.run();

	return 0;
}
