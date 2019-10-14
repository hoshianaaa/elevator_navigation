#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>

class Action{
public:
	Action(geometry_msgs::Point sp, geometry_msgs::Point fp, bool rotate = false);

	bool get_robot_pose();
	void move();
	void rotate();
	bool object();
	bool on_goal();
	void publish_vel(double speed, double ang_speed);

	double calc_goal_angle(geometry_msgs::Point start_point, geometry_msgs::Point finish_point, bool rotate = false);
	double calc_distance_line(geometry_msgs::Point start_point, geometry_msgs::Point finish_point, geometry_msgs::Point robot_point);
	double calc_angToline(geometry_msgs::Point start_point, geometry_msgs::Point finish_point, geometry_msgs::Point robot_point);

	
private:
	void scanCallback(const sensor_msgs::LaserScanPtr& msg);
	ros::NodeHandle n_;
	ros::Publisher velocity_pub_;
	ros::Subscriber scan_sub_;

	tf::TransformListener tf_listener_;
	tf::StampedTransform trans_;

	double robot_x_;
	double robot_y_;
	double robot_yaw_;
	geometry_msgs::Point goal_point_;
	geometry_msgs::Point start_point_;

	std::string robot_frame_, global_frame_;

	bool rotate_on_goal_;
	
	double speed_;

};

