#include <elevator_waypoints_nav/panel_action.h>

PanelAction::PanelAction()
{
	std::cout << "panel action class" << std::endl;
	velocity_pub_ = n_.advertise<geometry_msgs::Twist>("icart_mini/cmd_vel", 1);
	darknet_sub_ = n_.subscribe("darknet_data", 1, &PanelAction::darknetCallback, this);
	scan_sub_ = n_.subscribe("scan", 1, &PanelAction::scanCallback, this);
	
	darknet_lock_ = 0;
	darknet_data_ = 0;
	robot_frame_  = "base_link";
	global_frame_ = "odom";
	freq_ = 10;
	min_scan_ = 10000;

	get_robot_pose();
	home_point_ = robot_point_;
}



void PanelAction::darknetCallback(const std_msgs::Int32Ptr& msg){
	std::cout << "darknet callback" << std::endl;
	darknet_lock_ = 0;
	darknet_data_ = msg->data;
}

void PanelAction::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	
/*	
	if(!tf_listener_.waitForTransform(
		msg->header.frame_id,
		"/base_link",
		msg->header.stamp + ros::Duration().fromSec(msg->ranges.size()*msg->time_increment),
		ros::Duration(1.0))){
	std::cout << "scan callback transform error" << std::endl;	
	return;
	}
	
	sensor_msgs::PointCloud2 cloud2;
	
  	projector_.transformLaserScanToPointCloud("/base_link",*msg,
          cloud2,tf_listener_);
	
	pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
	pcl::fromROSMsg(cloud2, pcl_cloud);
*/
	min_scan_ = 10000;
	for (int i=0;i<msg->ranges.size();i++){
		double d = msg->ranges[i];
		if (d > 0.1 && d < min_scan_)min_scan_ = d;
	}
	std::cout << "min_scan:" << min_scan_ << std::endl;
}

bool PanelAction::rotate(double angle)
{
	ros::Rate loop_rate(freq_);
	get_robot_pose();
	double start_angle = robot_point_.z;
	double target_angle = start_angle + angle; 

	if (target_angle > M_PI)target_angle -= 2*M_PI;
	else if (target_angle < -M_PI)target_angle += 2*M_PI;


	while(!((robot_point_.z > target_angle - 0.03) && (robot_point_.z < target_angle + 0.03))){
		get_robot_pose();
		std::cout << "target angle:" << target_angle;
		std::cout << " now :" << robot_point_.z;	
		geometry_msgs::Twist vel;
		if (angle > 0)vel.angular.z = 0.1;
		else vel.angular.z = -0.1;
		velocity_pub_.publish(vel);
		std::cout << " ang vel :" << vel.angular.z << std::endl;	
		loop_rate.sleep();
		ros::spinOnce();
}
	
}

bool PanelAction::darknet_wait(long wait_time){
	darknet_lock_ = 1;
	long begin = ros::Time::now().sec;
	std::cout << "begin:" << begin << std::endl;
	while(1){
		ros::spinOnce();
		long timer = ros::Time::now().sec - begin;
		std::cout << "timer:" << timer << std::endl;
		std::cout << "darknet lock" << std::endl;
		if (!darknet_lock_){
			std::cout << "found panel" << std::endl;
			return 1;
		}
		else if(timer > wait_time){
			std::cout << "time out" << std::endl;
			return 0;
		}
	}
}

bool PanelAction::go_panel(double stop_distance){
	ros::Rate loop_rate(freq_);
	while(1){
		if (min_scan_ < stop_distance)break;
		geometry_msgs::Twist vel;
		vel.linear.x = 0.1;
		velocity_pub_.publish(vel);
		std::cout << "straight" << std::endl;
		loop_rate.sleep();
		ros::spinOnce();
	}
}

bool PanelAction::back_home(){
	ros::Rate loop_rate(freq_);
	double stop_distance = 0.5;
	while(1){
		get_robot_pose();
		double dx = robot_point_.x - home_point_.x;
		double dy = robot_point_.y - home_point_.y;
		double now_dis = std::sqrt(dx * dx + dy * dy);
		if (now_dis < stop_distance)break;
		geometry_msgs::Twist vel;
		vel.linear.x = -0.1;
		velocity_pub_.publish(vel);
		std::cout << "straight" << std::endl;
		loop_rate.sleep();
		ros::spinOnce();
	}
}


void PanelAction::run(bool inverse)
{
	int counter = 0;
	double k = -0.0002;
	while(1){
		if (darknet_wait(5))break;
		if (inverse)rotate(-M_PI/8);
		else rotate(M_PI/8);
	}
	std::cout << "darknet_data rotate:" << darknet_data_ * k << std::endl;
	rotate(darknet_data_ * k);
	go_panel(0.2);	
	back_home();
	std::cout << "rp:" << robot_point_.z << "hp:" << home_point_.z << std::endl;
	if (inverse)rotate(home_point_.z - robot_point_.z);
	else rotate(home_point_.z - robot_point_.z);
}

bool PanelAction::get_robot_pose(){
        try
        {
                tf::StampedTransform trans;
                tf_listener_.waitForTransform(global_frame_, robot_frame_, ros::Time(0), ros::Duration(3.0));
                tf_listener_.lookupTransform(global_frame_, robot_frame_, ros::Time(0), trans);
                robot_point_.x = trans.getOrigin().x();
                robot_point_.y = trans.getOrigin().y();
                robot_point_.z = tf::getYaw(trans.getRotation());
                return true;
        }
        catch(tf::TransformException &e)
        {
                ROS_WARN("%s", e.what());
                return false;
        }
}

