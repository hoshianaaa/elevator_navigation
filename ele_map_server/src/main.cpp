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

#include <ros/package.h>

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
		bool costmap;
		int number;
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
		EleMapServer(const std::string& dirname, int start_floor)
		{
			floor_number_ = start_floor;
			dirname_ = dirname;
			floor_number_sub = n.subscribe<std_msgs::Int8>("change_floor/floor_number", 1, &EleMapServer::floorNumberCallback,this);
			map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
			map_metadata_pub = n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
			map_for_costmap_pub = n.advertise<nav_msgs::OccupancyGrid>("map_for_costmap", 1, true);
			map_metadata_for_costmap_pub = n.advertise<nav_msgs::OccupancyGrid>("map_meta_data_for_costmap", 1, true);
			
			ros::NodeHandle private_nh("~");
      private_nh.param("start_floor", start_floor, start_floor);

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

			for (int i=0;i<floor_number_vector_.size();i++){
				std::stringstream ss;
				ss << floor_number_vector_[i];
				AddMapData(dirname + ss.str() + ".yaml", int(floor_number_vector_[i]));
				AddMapData(dirname + ss.str() + "_for_costmap.yaml", int(floor_number_vector_[i]),true);
			}
			publishMapFromFloorNumber(start_floor,false);
			publishMapFromFloorNumber(start_floor,true);
			service = n.advertiseService("static_map", &EleMapServer::mapCallback, this);
		}

	private:
		ros::NodeHandle n;
		ros::Publisher map_pub;
		ros::Publisher map_metadata_pub;
		ros::Publisher map_for_costmap_pub;
		ros::Publisher map_metadata_for_costmap_pub;
		ros::Subscriber floor_number_sub;						
		ros::ServiceServer service;
		std::string dirname_;
		std::vector<int> floor_number_vector_;
		int floor_number_;
		std::vector<MapYamlData> map_yaml_data_vector_;
		nav_msgs::MapMetaData meta_data_message_;
		nav_msgs::GetMap::Response map_resp_;
		nav_msgs::GetMap::Response map_resp_for_costmap_;
		void floorNumberCallback(const std_msgs::Int8::ConstPtr& msg)
		{
			ROS_INFO("change floor_number to [%d]", msg->data);
			publishMapFromFloorNumber(msg->data,false);
			publishMapFromFloorNumber(msg->data,true);
			floor_number_ = msg->data;
		}

		bool mapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res )
		{
			res = map_resp_;
			ROS_INFO("Sending map");
			return true;
		}

		bool AddMapData(const std::string &filename, const int map_number, bool costmap=false)
		{
			ROS_INFO("read file:[%s]\n",filename.c_str());
			try{
				std::ifstream fin(filename.c_str(), std::ifstream::in);
				if(fin.good() == false){
					ROS_ERROR("file name is bad");
					return false;
				}
				MapYamlData map_data;
				map_data.number = map_number;
				map_data.costmap = costmap;

				YAML::Node doc;

				#ifdef NEW_YAMLCPP
					doc = YAML::Load(fin);
				#else
					YAML::Parser parser(fin);
					parser.GetNextDocument(doc);
				#endif
				try{
					doc["image"] >> map_data.image;
					map_data.image = dirname_ + map_data.image;
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

		bool getIndex(const int number, int& index, bool costmap=false){
			for (int i=0;i<map_yaml_data_vector_.size();i++){
				if (map_yaml_data_vector_[i].number == number){
					if (costmap == map_yaml_data_vector_[i].costmap)
					{
						index = i;
						return true;
					}
				}
				if (i == map_yaml_data_vector_.size() - 1){
					ROS_ERROR("Nothin number %d in elevator map\n", number);
					return false;
				}
			}
		}

		void getMapDataFromIndex(const int index, std::string& image, double& resolution, double* origin, int& negate, double& occupied_thresh, double& free_thresh)
		{
			ROS_INFO("get map data from %d\n", index);
			image = map_yaml_data_vector_[index].image;
			resolution = map_yaml_data_vector_[index].resolution;
			origin[0] = map_yaml_data_vector_[index].origin[0];
			origin[1] = map_yaml_data_vector_[index].origin[1];
			origin[2] = map_yaml_data_vector_[index].origin[2];
			negate = map_yaml_data_vector_[index].negate;
			occupied_thresh = map_yaml_data_vector_[index].occupied_thresh;
			free_thresh = map_yaml_data_vector_[index].free_thresh;
		}

		bool publishMapFromFloorNumber(const int floor_number, const int costmap=false){

			int index;
			getIndex(floor_number, index, costmap);
			MapYamlData md;
			getMapDataFromIndex(index, md.image, md.resolution, md.origin, md.negate, md.occupied_thresh, md.free_thresh);

			if(costmap){
				map_server::loadMapFromFile(&map_resp_for_costmap_,md.image.c_str(), md.resolution, md.negate,md.occupied_thresh, md.free_thresh, md.origin);
				ROS_INFO("Loading map from image \"%s\"", md.image.c_str());
				map_resp_for_costmap_.map.info.map_load_time = ros::Time::now();
				map_resp_for_costmap_.map.header.frame_id = "map_for_costmap";
				map_resp_for_costmap_.map.header.stamp = ros::Time::now();
				meta_data_message_ = map_resp_for_costmap_.map.info;
				//map_metadata_for_costmap_pub.publish(meta_data_message_);
				map_for_costmap_pub.publish(map_resp_for_costmap_.map);

				return true;
			}

			map_server::loadMapFromFile(&map_resp_,md.image.c_str(), md.resolution, md.negate,md.occupied_thresh, md.free_thresh, md.origin);
			ROS_INFO("Loading map from image \"%s\"", md.image.c_str());
			map_resp_.map.info.map_load_time = ros::Time::now();
			map_resp_.map.header.frame_id = "map";
			map_resp_.map.header.stamp = ros::Time::now();
			meta_data_message_ = map_resp_.map.info;
			map_metadata_pub.publish(meta_data_message_);
			map_pub.publish(map_resp_.map);
			return true;
		}
	};
	

int main(int argc, char** argv){
	ros::init(argc, argv, "ele_map_server");
  std::string path = ros::package::getPath("ele_map_server");
  std::string map_path = path + "/elevator_map/";
	EleMapServer es(map_path, 18);
	ros::Rate r(10);
	while(ros::ok()){
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
