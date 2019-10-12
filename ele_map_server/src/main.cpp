/* Author: Hoshianaaa */

#include <ros/ros.h>

#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int8.h>

#include <fstream>

#include <map_server/image_loader.h>
#include <yaml-cpp/yaml.h>
#include <stdio.h>
#include <string>
#include <dirent.h>
#include <sstream>

#ifdef NEW_YAMLCPP
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
	i = node.as<T>();
}
#endif

class MapYamlData
{
	public:
		std::string image;
		double resolution;
		double origin[3];
		int negate;
		double occupied_thresh;
		double free_thresh;
};

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

			DIR *dp = opendir(dirname.c_str());
			if (dp!=NULL)
			{
					struct dirent* dent;
					do{
						dent = readdir(dp);
						if (dent!=NULL)
							{
								std::string str = std::string(dent->d_name);
								std::cout<<str<<std::endl;
								
								int count = 0;
								for (int i=0;i<str.length();i++){
									if (str[i] == '.')
									{
										if (count){
											if (str[i+1] == 'y')
											{
												int number = atoi((str.substr(0,count)).c_str());
												floor_number_vector_.push_back(number);
											}
										}
										break;
									}
									if (str[i] == '_')break;
									count++;
								}
							}
					}while(dent!=NULL);
			}
			std::cout << "floor number vector" << std::endl;
			for (int i=0;i<floor_number_vector_.size();i++){
				std::cout << floor_number_vector_[i] << std::endl;
				std::stringstream ss;
				ss << floor_number_vector_[i];
				AddMapData(dirname + ss.str() + ".yaml");
				AddMapData(dirname + ss.str() + "_for_costmap.yaml");
			}
			std::cout << std::endl;
			
		}

	private:
		ros::NodeHandle n;
		ros::Publisher map_pub;
		ros::Publisher map_metadata_pub;
		ros::Publisher map_metadata_for_costmap_pub;
		ros::Publisher map_for_costmap_pub;
		ros::Subscriber floor_number_sub;						
		std::vector<int> floor_number_vector_;
		std::vector<MapYamlData> map_yaml_data_vector_;
		void floorNumberCallback(const std_msgs::Int8::ConstPtr& msg)
		{
			ROS_INFO("change floor_number to [%d]", msg->data);
		}

		bool AddMapData(const std::string &filename)
		{
			ROS_INFO("read file:[%s]\n",filename.c_str());
			try{
				std::ifstream fin(filename.c_str(), std::ifstream::in);
				if(fin.good() == false){
					ROS_ERROR("file name is bad");
					return false;
				}
				MapYamlData map_data;

				YAML::Node doc;

				#ifdef NEW_YAMLCPP
					doc = YAML::Load(fin);
				#else
					YAML::Parser parser(fin);
					parser.GetNextDocument(doc);
				#endif
				try{
					doc["image"] >> map_data.image;
				} catch (YAML::InvalidScalar) {
					ROS_ERROR("The map does not contain an image tag or it is invalid.");
				}
				try {
					doc["resolution"] >> map_data.resolution;
				} catch (YAML::InvalidScalar) {
					ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
				}
				try {
					doc["origin"][0] >> map_data.origin[0];
					doc["origin"][1] >> map_data.origin[1];
					doc["origin"][2] >> map_data.origin[2];
				} catch (YAML::InvalidScalar) {
					ROS_ERROR("The map dose not contain an origin tag or it is invalid.");
				}
				try {
					doc["negate"] >> map_data.negate;
				} catch (YAML::InvalidScalar) {
					ROS_ERROR("The map does not contain an negate tag or it is invalid.");
				}

				try {
				doc["occupied_thresh"] >> map_data.occupied_thresh;
				} catch (YAML::InvalidScalar) {
					ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
				}
				
				try {
				doc["free_thresh"] >> map_data.free_thresh;
				} catch (YAML::InvalidScalar) {
					ROS_ERROR("The map does not contain an free_thresh tag or it is invalid.");
				}
				map_yaml_data_vector_.push_back(map_data);
			}catch(YAML::ParserException &e){
				ROS_ERROR("paser exception");
				return false;
			}catch(YAML::RepresentationException &e){
				ROS_ERROR("representation exception");
				std::cout << e.what() << std::endl;
				return false;
			}

			ROS_INFO("add map data");
			return true;
		}
};
	

int main(int argc, char** argv){
	ros::init(argc, argv, "ele_map_server");
	EleMapServer es("/home/icart/catkin_ws/src/elevator_navigation/ele_map_server/elevator_map/", 1);
	return 0;
}
