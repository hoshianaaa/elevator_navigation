#include <elevator_waypoints_nav/waypoints_nav.h>

#ifdef NEW_YAMLCPP
templete<typename T>
void operator >> (const YAML::Node& node, T& i)
{
	i = node.as<T>();
};
#endif

WaypointsNavigation::WaypointsNavigation() :
	move_base_action_("move_base", true),
	rate_(10)
{
	while((move_base_action_.waitForServer(ros::Duration(1.0)) == false) && (ros::ok() == true))
	{
		ROS_INFO("waiting...");
	}

	world_frame_ = "map";
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

void WaypointsNavigation::sleep(){
	rate_.sleep();
	ros::spinOnce();
}

void WaypointsNavigation::run(){
	while(ros::ok()){
		sleep();
	}
}

bool readFile(const std::string &filename){
	waypoints_.poses.clear();
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

			geometry_msgs::Pose pose;
			if(wp_node != NULL){
					for(int i=0; i < wp_node->size(); i++){

							(*wp_node)[i]["point"]["x"] >> pose.position.x;
							(*wp_node)[i]["point"]["y"] >> pose.position.y;
							(*wp_node)[i]["point"]["z"] >> pose.position.z;
							(*wp_node)[i]["point"]["floor"] >> pose.position.z;


							waypoints_.poses.push_back(pose);

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
					(*fp_node)["pose"]["position"]["x"] >> pose.position.x;
					(*fp_node)["pose"]["position"]["y"] >> pose.position.y;
					(*fp_node)["pose"]["position"]["z"] >> pose.position.z;
					(*fp_node)["pose"]["position"]["floor"] >> pose.position.z;

					(*fp_node)["pose"]["orientation"]["x"] >> pose.orientation.x;
					(*fp_node)["pose"]["orientation"]["y"] >> pose.orientation.y;
					(*fp_node)["pose"]["orientation"]["z"] >> pose.orientation.z;
					(*fp_node)["pose"]["orientation"]["w"] >> pose.orientation.w;

					waypoints_.poses.push_back(pose);

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

	geometry_msgs::Point p;
	p.x = -3;
	w_nav.startNavigationGL(p);
	w_nav.sleep();
	w_nav.run();

	return 0;
}
