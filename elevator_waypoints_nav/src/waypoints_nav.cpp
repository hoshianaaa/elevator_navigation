#include <ros/ros.h>
#include <elevator_waypoints_nav/waypoints_nav.h>
#include <ros/package.h>
#include <elevator_navigation_srv/ElevatorAction.h>

WaypointsNavigation::WaypointsNavigation() :
	move_base_action_("move_base", true),
	rate_(10),
	dist_err_(0.8)
{


	while((move_base_action_.waitForServer(ros::Duration(1.0)) == false) && (ros::ok() == true))
	{
		ROS_INFO("waiting...");
	}

	std::string filename, pkg_path;

  pkg_path = ros::package::getPath("elevator_waypoints_nav");
	filename = pkg_path + "/waypoints_cfg/waypoints.yaml";


	ros::NodeHandle nh;
  elevator_action_client_ = nh.serviceClient<elevator_navigation_srv::ElevatorAction>("elevator_action");

	ros::NodeHandle private_nh("~");

	private_nh.param("filename", filename, filename);
	ROS_INFO_STREAM("Read waypoints data from " << filename);
	if(!readFile(filename)) {
			ROS_ERROR("Failed loading waypoints file");
	}

	world_frame_ = "map";
	robot_frame_ = "base_link";

	elevator_front_pose_.position.x = 3;
	elevator_front_pose_.position.y = 0;
	elevator_front_pose_.orientation = tf::createQuaternionMsgFromYaw(M_PI);

	start_floor_ = 3;

	elevator_point_.x = 0;
	elevator_point_.y = 0;

	current_waypoint_ = waypoints_.begin();
	finish_waypoint_ = waypoints_.end() - 1;
	computeWpOrientation();

	dist_err_ = 0.5;
}


void WaypointsNavigation::startNavigationGL(const geometry_msgs::Point &dest){
	geometry_msgs::Pose pose;
	pose.position = dest;
	pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
	startNavigationGL(pose);
}

void WaypointsNavigation::startNavigationGL(const geometry_msgs::Pose &dest){
	move_base_msgs::MoveBaseGoal move_base_goal;
	move_base_goal.target_pose.header.stamp = ros::Time::now(); move_base_goal.target_pose.header.frame_id = world_frame_;
	move_base_goal.target_pose.pose.position = dest.position;
	move_base_goal.target_pose.pose.orientation = dest.orientation;
	move_base_action_.sendGoal(move_base_goal);
	std::cout << "send goal" << std::endl;
}

void WaypointsNavigation::startNavigationGL(const Waypoint &wp){
	startNavigationGL(wp.geometry_msg);
}

void WaypointsNavigation::computeWpOrientation(){
	double goal_direction;
	for (std::vector<Waypoint>::iterator it = waypoints_.begin(); it != finish_waypoint_; it++) {
		std::cout << it->floor << std::endl;
		if ((it)->floor != (it+1)->floor){
			goal_direction = atan2(elevator_front_pose_.position.y - (it)->geometry_msg.position.y,
																		elevator_front_pose_.position.x - (it)->geometry_msg.position.x);
		}
		else
		{
			goal_direction = atan2((it+1)->geometry_msg.position.y - (it)->geometry_msg.position.y,
														 (it+1)->geometry_msg.position.x - (it)->geometry_msg.position.x);
		}
		(it)->geometry_msg.orientation = tf::createQuaternionMsgFromYaw(goal_direction);
	}
}

bool WaypointsNavigation::onNavigationPoint(const geometry_msgs::Point &dest, double dist_err = 0.8){
	tf::StampedTransform robot_gl = getRobotPosGL();

	const double wx = dest.x;
	const double wy = dest.y;
	const double rx = robot_gl.getOrigin().x();
	const double ry = robot_gl.getOrigin().y();
	const double dist = std::sqrt(std::pow(wx - rx, 2) + std::pow(wy - ry, 2));

	return dist < dist_err;
}


void WaypointsNavigation::sleep(){
	rate_.sleep();
	ros::spinOnce();
}

void WaypointsNavigation::elevator_action(){
  elevator_navigation_srv::ElevatorAction srv;
  elevator_action_client_.call(srv);
	}

bool WaypointsNavigation::readFile(const std::string &filename){
	waypoints_.clear();
	try{
			std::ifstream ifs(filename.c_str(), std::ifstream::in);
			if(ifs.good() == false){
					return false;
			}

			YAML::Node node;
			
			#ifdef NEW_YAMLCPP
					node = YAML::Load(ifs);
			#else
					YAML::Parser parser(ifs);
					parser.GetNextDocument(node);
			#endif

			#ifdef NEW_YAMLCPP
					const YAML::Node &wp_node_tmp = node["waypoints"];
					const YAML::Node *wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
			#else
					const YAML::Node *wp_node = node.FindValue("waypoints");
			#endif

			Waypoint wp;
			if(wp_node != NULL){
					for(int i=0; i < wp_node->size(); i++){

							(*wp_node)[i]["point"]["x"] >> wp.geometry_msg.position.x;
							(*wp_node)[i]["point"]["y"] >> wp.geometry_msg.position.y;
							(*wp_node)[i]["point"]["z"] >> wp.geometry_msg.position.z;
							(*wp_node)[i]["point"]["floor"] >> wp.floor;


							waypoints_.push_back(wp);

					}
			}else{
					return false;
			}
			
			#ifdef NEW_YAMLCPP
					const YAML::Node &fp_node_tmp = node["finish_pose"];
					const YAML::Node *fp_node = fp_node_tmp ? &fp_node_tmp : NULL;
			#else
					const YAML::Node *fp_node = node.FindValue("finish_pose");
			#endif

			if(fp_node != NULL){
					(*fp_node)["pose"]["position"]["x"] >> wp.geometry_msg.position.x;
					(*fp_node)["pose"]["position"]["y"] >> wp.geometry_msg.position.y;
					(*fp_node)["pose"]["position"]["z"] >> wp.geometry_msg.position.z;
					(*fp_node)["pose"]["position"]["floor"] >> wp.floor;

					(*fp_node)["pose"]["orientation"]["x"] >> wp.geometry_msg.orientation.x;
					(*fp_node)["pose"]["orientation"]["y"] >> wp.geometry_msg.orientation.y;
					(*fp_node)["pose"]["orientation"]["z"] >> wp.geometry_msg.orientation.z;
					(*fp_node)["pose"]["orientation"]["w"] >> wp.geometry_msg.orientation.w;

					waypoints_.push_back(wp);

			}else{
					return false;
			}

	}catch(YAML::ParserException &e){
			return false;
	}catch(YAML::RepresentationException &e){
			return false;
	}

	return true;
}

tf::StampedTransform WaypointsNavigation::getRobotPosGL(){
	tf::StampedTransform robot_gl;
	try{
		tf_listener_.waitForTransform(world_frame_, robot_frame_, ros::Time(0.0), ros::Duration(1.0));
		tf_listener_.lookupTransform(world_frame_, robot_frame_, ros::Time(0.0), robot_gl);
	}catch(tf::TransformException &e){
		ROS_WARN_STREAM("tf::TransformException: " << e.what());
	}

	return robot_gl;
}

void WaypointsNavigation::run(){

	int now_floor, next_floor;

	now_floor = start_floor_;
	while(ros::ok()){

		next_floor = current_waypoint_->floor;

		if (now_floor == next_floor){
			startNavigationGL(*current_waypoint_);
			while(!onNavigationPoint(current_waypoint_->geometry_msg.position, dist_err_)) {}

		} else {

			startNavigationGL(elevator_front_pose_);
			while(!onNavigationPoint(elevator_front_pose_.position, dist_err_)) {}
			move_base_action_.cancelAllGoals();
			elevator_action();
			startNavigationGL(*current_waypoint_);
			while(!onNavigationPoint(current_waypoint_->geometry_msg.position, dist_err_)) {}

			now_floor = current_waypoint_->floor;
		}

		if (current_waypoint_ == finish_waypoint_){
			std::cout << "finish" << std::endl;
			break;
		}
		current_waypoint_++;
	}
}


int main(int argc, char *argv[]){
	ros::init(argc, argv, ROS_PACKAGE_NAME);
	WaypointsNavigation w_nav;
	w_nav.run();
	return 0;
}
