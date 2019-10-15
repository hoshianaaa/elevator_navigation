#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

class WaypointsNavigation{
public:
	WaypointsNavigation();
	void startNavigationGL(const geometry_msgs::Point &dest);
	void startNavigationGL(const geometry_msgs::Pose &dest);
	void sleep();
	void run();
private:
	std::string world_frame_, robot_frame_;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_;
	ros::Rate rate_;
	tf::TransformListener tf_listener_;
};

