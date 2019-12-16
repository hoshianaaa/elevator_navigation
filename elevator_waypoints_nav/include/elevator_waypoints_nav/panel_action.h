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

#include <elevator_navigation_srv/ArmMotion.h>

class PanelAction{
public:
	PanelAction();
	bool rotate_for_target_angle(double target_angle);
	bool rotate(double angle);
	bool rotate_home();
	bool straight(double d);
  bool line_tracking_stop_point(double x, double y, double angle);
	bool go_panel(double stop_distance);
	bool back(double distance, double speed=0.1);
  bool rotate_for_bounding_box(const int bounding_box_target_x);
  bool up_arm(double height, double error_th);
  bool home_arm();
  int get_bounding_box_state();
  double get_scan_sum();

private:
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void boundingBoxCallback(const geometry_msgs::Point::ConstPtr& msg);
	ros::NodeHandle n_;
	ros::Publisher velocity_pub_;
	ros::Subscriber scan_sub_;
	ros::Subscriber bounding_box_sub_;
	ros::ServiceClient arm_motion_client_;
	
	tf::TransformListener tf_listener_;
	std::string robot_frame_, global_frame_, ar_marker_frame_;
	
	geometry_msgs::Point robot_point_, home_point_;
	laser_geometry::LaserProjection projector_;
	
	bool scan_lock_;
	double min_scan_;
	bool get_robot_pose();
	bool get_ar_marker_pose(double& height);
  bool fix_angle(double& angle);
  double calc_distance(double x, double y);
  double calc_inner_product(double x1, double y1, double x2, double y2);
	double freq_;

  int bounding_box_x_;
  int bounding_box_lock_;
  int bounding_box_state_;

  double scan_sum_;
};
