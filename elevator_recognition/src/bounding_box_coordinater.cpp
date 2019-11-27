#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <limits>
#include <boost/thread/thread.hpp>

class boundingBoxCoordinater
{
public:
  boundingBoxCoordinater();
  void run();
private:
  ros::NodeHandle nh_;
  tf::TransformBroadcaster br_;
  tf::Transform transform_;
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
  int tf_broad_rate = 10;
}

void boundingBoxCoordinater::cloudCallback(const sensor_msgs::PointCloud2ConstPtr&msgs){

  cloud_ = *msgs;
}

void boundingBoxCoordinater::boundingBoxPosCallback(const geometry_msgs::PointConstPtr& msgs){
  std::cout << "bounding box callback" << std::endl;  

  int bx = msgs->x;
  int by = msgs->y;

  float x, y, z;
  while(1){
    getCloudPoint(bx, by, x, y, z);
    if (!isnan(x))break;
    bx++;
  }
  std::cout << "x:" << x << " y:" << y << " z:" << z << std::endl;
  
  transform_.setOrigin( tf::Vector3(x, y, z) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0); 
  transform_.setRotation(q);
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

void boundingBoxCoordinater::run(){
  ros::Rate r(10);
  while (ros::ok())
  {
    br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), sensor_frame_, target_frame_));
    std::cout << "run" << std::endl;  
    ros::spinOnce();
    r.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bounding_box_coordinater");
  boundingBoxCoordinater bc;
  bc.run();
  return 0;
}
