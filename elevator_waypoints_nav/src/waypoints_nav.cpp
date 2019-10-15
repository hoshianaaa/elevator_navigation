#include <elevator_waypoints_nav/waypoints_nav.h>

WaypointsNavigation::WaypointsNavigation() :
	move_base_action_("move_base", true),
	rate_(10)
{

	std::cout << "test" << std::endl;
	while((move_base_action_.waitForServer(ros::Duration(1.0)) == false) && (ros::ok() == true))
	{
		ROS_INFO("waiting...");
	}

	ros::NodeHandle private_nh("~");
	private_nh.param("filename", filename, filename)
	ROS_INFO_STREAM("Read waypoints data from " << filename);
	if(!readFile(filename)) {
			ROS_ERROR("Failed loading waypoints file");
	} else {
			last_waypoint_ = waypoints_.poses.end()-2;
			finish_pose_ = waypoints_.poses.end()-1;
			computeWpOrientation();
	}
		current_waypoint_ = waypoints_.poses.begin();
	} else {
			ROS_ERROR("waypoints file doesn't have name");
	}
	world_frame_ = "map";

	elevator_front_pose_.position.x = 3;
	elevator_front_pose_.position.y = 0;
	elevator_front_pose_.orientation = tf::createQuaternionMsgFromYaw(M_PI);

	elevator_point_.x = 0;
	elevator_point_.y = 0;


}


void WaypointsNavigation::startNavigationGL(const geometry_msgs::Point &dest){
	geometry_msgs::Pose pose;
	pose.position = dest;
	pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
	startNavigationGL(pose);
}

void WaypointsNavigation::startNavigationGL(const geometry_msgs::Pose &dest){
	move_base_msgs::MoveBaseGoal move_base_goal;
	move_base_goal.target_pose.header.stamp = ros::Time::now();
	move_base_goal.target_pose.header.frame_id = world_frame_;
	move_base_goal.target_pose.pose.position = dest.position;
	move_base_goal.target_pose.pose.orientation = dest.orientation;
	move_base_action_.sendGoal(move_base_goal);
	std::cout << "send goal" << std::endl;
}

void computeWpOrientation(){
	double goal_direction;
	for (std::vector<Waypoint>::iterator it = waypoints_.begin(); it != finish_pose_; it++) {
		if ((it)->floor != (it+1)->floor){
			goal_direction = atan2(elevator_front_pose_.position.y - (it)->geometry_msg.position.y,
																		elevator_front_pose_.position.x - (it)->position.x);
		}
		else
		{
			goal_direction = atan2((it+1)->geometry_msg.position.y - (it)->geometry_msg.position.y,
														 (it+1)->geometry_msg.position.x - (it)->geometry_msg.position.x);
		}
		(it)->orientation = tf::createQuaternionMsgFromYaw(goal_direction);
		waypoints_.geometry_msg.header.frame_id = world_frame_;
	}
}

void WaypointsNavigation::sleep(){
	rate_.sleep();
	ros::spinOnce();
}

void WaypointsNavigation::run(){
	while(ros::ok()){



		sleep();
	}
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


int main(int argc, char *argv[]){
	ros::init(argc, argv, ROS_PACKAGE_NAME);
	WaypointsNavigation w_nav;
	return 0;
}
