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
	bool rotate(double speed, double goal_angel_th);
	bool StopRobot();

	bool on_goal();

	void publish_vel(double speed, double ang_speed); bool calc_dis_diff(const geometry_msgs::Point start_point, const geometry_msgs::Point finish_point, const geometry_msgs::Point robot_point, double& distance); bool calc_ang_diff(const geometry_msgs::Point start_point, const geometry_msgs::Point finish_point, const geometry_msgs::Point robot_point, double& in_ang);
	bool calc_pose_diff(const double robot_yaw, double& ang);

	bool calc_vectors_interior_angle(const double a1, const double a2, const double b1, const double b2, double& ang);
	bool calc_goal_angle(const geometry_msgs::Point start_point, const geometry_msgs::Point finish_point, const bool rotate, double& ang);

	void print_env_data();
	void print_robot_data();

	
private:
	void scanCallback(const sensor_msgs::LaserScanPtr& msg);
	ros::NodeHandle n_;
	ros::Publisher velocity_pub_;
	ros::Subscriber scan_sub_;

	tf::TransformListener tf_listener_;
	tf::StampedTransform trans_;

	geometry_msgs::Point goal_point_;
	geometry_msgs::Point start_point_;
	geometry_msgs::Point robot_point_;
	double goal_angle_;

	std::string robot_frame_, global_frame_;

	bool rotate_on_goal_;
	
	double speed_;
	double ang_speed_max_;
	double freq_;

	double sensor_range_;
	double sensor_data_min_;

};

