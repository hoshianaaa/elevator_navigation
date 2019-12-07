#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <laser_geometry/laser_geometry.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


class PanelAction{
public:
	PanelAction();
	void run(bool inverse);
	bool rotate(double angle);
	bool straight(double d);
	bool go_panel(double stop_distance);
	bool back_home();

private:
	void darknetCallback(const std_msgs::Int32Ptr& msg);
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	ros::NodeHandle n_;
	ros::Publisher velocity_pub_;
	ros::Subscriber darknet_sub_;
	ros::Subscriber scan_sub_;
	
	tf::TransformListener tf_listener_;
	std::string robot_frame_, global_frame_;
	
	geometry_msgs::Point robot_point_, home_point_;
	laser_geometry::LaserProjection projector_;
	

	bool darknet_wait(long wait_time);
	bool darknet_lock_;
	bool scan_lock_;
	int darknet_data_;

	double min_scan_;

	bool get_robot_pose();

	double freq_;
};
