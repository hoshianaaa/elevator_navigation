/* Author: Hoshianaaa */

#include <ros/ros.h>

#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int8.h>

#include <fstream>

#include <map_server/image_loader.h>
#include <yaml-cpp/yaml.h>
#include <dirent.h>


class EleMapServer
{
	public:
		EleMapServer(const std::string& dirname, int start_number)
		{
			floor_number_sub = n.subscribe<std_msgs::Int8>("floor_number", 1, &EleMapServer::floorNumberCallback,this);
			map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
			map_metadata_pub = n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
			map_for_costmap_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
			map_metadata_for_costmap_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
			
			ros::NodeHandle private_nh("~");

		}

	private:
		ros::NodeHandle n;
		ros::Publisher map_pub;
		ros::Publisher map_metadata_pub;
		ros::Publisher map_metadata_for_costmap_pub;
		ros::Publisher map_for_costmap_pub;
		ros::Subscriber floor_number_sub;						
		void floorNumberCallback(const std_msgs::Int8::ConstPtr& msg)
		{
			ROS_INFO("change floor_number to [%d]", msg->data);
		}
};
	

int main(int argc, char** argv){
	ros::init(argc, argv, "ele_map_server");
	EleMapServer es("/home/icart/jiritu_soukou/map/elevator", 1);
	return 0;
}
