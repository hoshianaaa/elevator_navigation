#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <std_msgs/Int32.h>

class SearchPanel{
public:
	SearchPanel();
	void run();
	
private:
	void darknetCallback(const std_msgs::Int32Ptr& msg);
	ros::NodeHandle n_;
	ros::Publisher velocity_pub_;
	ros::Subscriber darknet_sub_;
	
	tf::TransformListener tf_listener_;
	std::string robot_frame_, global_frame_;
	
	geometry_msgs::Point robot_point_;

	bool rotate();
	bool darknet_wait();
	bool darknet_lock_;

	bool get_robot_pose();
};
