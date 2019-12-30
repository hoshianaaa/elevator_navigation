#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <laser_geometry/laser_geometry.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <elevator_navigation_msgs/ArmMotionAction.h>
#include <actionlib/client/simple_action_client.h>


typedef actionlib::SimpleActionClient<elevator_navigation_msgs::ArmMotionAction> ArmMotionClient;

class BoundingBox{
public:
  int x;
  int id;
};

class PanelAction{
public:
	PanelAction(std::string name = "");
	bool rotate_for_target_angle(double target_angle);
	bool rotate(double angle);
	bool rotate_home();
	bool straight(double d);
  bool line_tracking_stop_point(double x, double y, double angle);
	bool go_panel(double stop_distance);
	bool go_target_motion(double x, double y, double stop_distance);
	bool back(double distance, double speed=0.1);
  bool rotate_for_bounding_box(const int bounding_box_target_x = 330, const int target_id = 0, const int error_th = 5);
  bool up_arm(double height, double error_th, int start_number = 4);
  bool down_arm(double height, double error_th, int start_number = 2);
  bool home_arm();
  bool home_arm_down();
  bool find_bounding_box(const int id = 1);
  double get_scan_sum();
  void track_b_box_start(int id);
  void track_b_box_clear();
  bool get_found_b_box_state();

  void check_door_start();
  void check_door_stop();
  bool stop_action();

private:
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void boundingBoxCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
	ros::NodeHandle n_;
	ros::Publisher velocity_pub_;
	ros::Subscriber scan_sub_;
	ros::Subscriber bounding_box_sub_;
  ArmMotionClient *arm_motion_client_, *arm_motion_down_client_;
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

  std::vector<BoundingBox> boxes_;
  int bounding_box_lock_;

  int search_b_box_;
  bool found_b_box_;
  int track_b_box_id_;

  double scan_sum_;
  double first_scan_sum_;

  int do_check_door_;
  int open_door_;
  std::string name_;

};
