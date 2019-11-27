#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <limits>

class boundingBoxCoordinater
{
public:
  boundingBoxCoordinater();
private:
  ros::NodeHandle nh_;
  tf::TransformBroadcaster br_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber bounding_box_pos_sub_;
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msgs);
  void boundingBoxPosCallback(const geometry_msgs::PointConstPtr& msgs);
  std::string cloud_topic_, bounding_box_pos_topic_;
  std::string sensor_frame_ , target_frame_;
  sensor_msgs::PointCloud2 cloud_;
  bool getCloudPoint(int x, int y,float &px, float &py, float &pz); //get cloud point on position (x, y) in image
};

boundingBoxCoordinater::boundingBoxCoordinater(){
  cloud_topic_ = "camera/depth/points";
  bounding_box_pos_topic_ = "bounding_box_pos";
  sensor_frame_ = "camera_depth_optical_frame";
  target_frame_ = "button";
  cloud_sub_ = nh_.subscribe(cloud_topic_, 1, &boundingBoxCoordinater::cloudCallback, this);
  bounding_box_pos_sub_ = nh_.subscribe(bounding_box_pos_topic_, 1, &boundingBoxCoordinater::boundingBoxPosCallback, this);
}

void boundingBoxCoordinater::cloudCallback(const sensor_msgs::PointCloud2ConstPtr&msgs){

  cloud_ = *msgs;
}

void boundingBoxCoordinater::boundingBoxPosCallback(const geometry_msgs::PointConstPtr& msgs){
  std::cout << "bounding box callback" << std::endl;  

  float x, y, z;
  getCloudPoint((int)msgs->x, (int)msgs->y, x, y, z);
  std::cout << "x:" << x << " y:" << y << " z:" << z << std::endl;
  
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(x, y, z) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0); 
  transform.setRotation(q);
  br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), sensor_frame_, target_frame_));


}

bool boundingBoxCoordinater::getCloudPoint(int x, int y, float &px, float &py, float &pz){

  int arrayPosX = y * cloud_.row_step + x * cloud_.point_step + cloud_.fields[0].offset;
  int arrayPosY = y * cloud_.row_step + x * cloud_.point_step + cloud_.fields[1].offset;
  int arrayPosZ = y * cloud_.row_step + x * cloud_.point_step + cloud_.fields[2].offset;

  memcpy(&px, &cloud_.data[arrayPosX], sizeof(float));
  memcpy(&py, &cloud_.data[arrayPosY], sizeof(float));
  memcpy(&pz, &cloud_.data[arrayPosZ], sizeof(float));

  return 1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bounding_box_coordinater");
  boundingBoxCoordinater bc;
  ros::Rate r(10);
  while(ros::ok()){
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
