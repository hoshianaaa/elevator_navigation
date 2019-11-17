#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

class PanelAction{
public:
	PanelAction();
	void run();
	
private:
	void darknetCallback(const std_msgs::Int32Ptr& msg);
	ros::NodeHandle n_;
	ros::Publisher velocity_pub_;
	ros::Subscriber darknet_sub_;

	bool get_robot_pose();
};
