#include <ros/ros.h>
#include <ros/package.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <elevator_waypoints_nav/action.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <fstream>

class Waypoint{
	public:
		geometry_msgs::Pose geometry_msg;
		int floor;
};

#ifdef NEW_YAMLCPP
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
	i = node.as<T>();
};
#endif

class WaypointsNavigation{
public:
	WaypointsNavigation();
	void startNavigationGL(const geometry_msgs::Point &dest);
	void startNavigationGL(const geometry_msgs::Pose &dest);
	void startNavigationGL(const Waypoint &wp);
	void computeWpOrientation();
	bool onNavigationPoint(const geometry_msgs::Point &dist, double dist_err);
	void sleep();
	bool readFile(const std::string &filename);
	void elevator_action();
	tf::StampedTransform getRobotPosGL();
	void run();
private:
	std::string world_frame_, robot_frame_;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_;
  ros::ServiceClient elevator_action_client_;
	ros::Rate rate_;
	tf::TransformListener tf_listener_;
	std::vector<Waypoint> waypoints_;
	std::vector<Waypoint>::iterator current_waypoint_;
	std::vector<Waypoint>::iterator finish_waypoint_;


	geometry_msgs::Pose elevator_front_pose_;
	geometry_msgs::Point elevator_point_;

	int start_floor_;

	double dist_err_;

};

