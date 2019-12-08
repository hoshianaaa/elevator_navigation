#include <elevator_waypoints_nav/panel_action.h>
PanelAction::PanelAction()
{
	std::cout << "panel action class" << std::endl;
	velocity_pub_ = n_.advertise<geometry_msgs::Twist>("icart_mini/cmd_vel", 1);
	scan_sub_ = n_.subscribe("scan", 1, &PanelAction::scanCallback, this);
	bounding_box_sub_ = n_.subscribe("bounding_box_pos", 1, &PanelAction::boundingBoxCallback, this);
  arm_motion_client_ = n_.serviceClient<elevator_navigation_srv::ArmMotion>("arm_motion");
	
	scan_lock_ = 1;
	robot_frame_  = "base_link";
	global_frame_ = "odom";
  ar_marker_frame_ = "ar_marker_187";
	freq_ = 10;
	min_scan_ = 0;

	get_robot_pose();
	home_point_ = robot_point_;

  bounding_box_x_ = 1000;
  bounding_box_lock_ = 4;
  bounding_box_state_ = 0;
}

void PanelAction::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  scan_lock_ = 0;
  min_scan_ = min_scan_ * 0.7 + msg->ranges[msg->ranges.size()/2] * 0.3;
}

bool PanelAction::rotate(double angle)
{
	ros::Rate loop_rate(freq_);
	get_robot_pose();
	double start_angle = robot_point_.z;
	double target_angle = start_angle + angle; 

	if (target_angle > M_PI)target_angle -= 2*M_PI;
	else if (target_angle < -M_PI)target_angle += 2*M_PI;

  int counter = 0;
	while(1){
    
    if((robot_point_.z > target_angle - 0.02) && (robot_point_.z < target_angle + 0.02))counter++;
    else counter = 0;
    if(counter>20)break;
    std::cout << counter << " " << robot_point_.z - target_angle << std::endl;

		get_robot_pose();
		std::cout << "target angle:" << target_angle;
		std::cout << " now :" << robot_point_.z;	
		geometry_msgs::Twist vel;
    double diff = target_angle - robot_point_.z;
    vel.angular.z = diff;
    if(vel.angular.z > 0.2)vel.angular.z = 0.2;
    if(vel.angular.z < -0.2)vel.angular.z = -0.2;
		velocity_pub_.publish(vel);
		std::cout << " ang vel :" << vel.angular.z << std::endl;	
		loop_rate.sleep();
		ros::spinOnce();
  }
}

bool PanelAction::straight(double d)
{
  int back = 0;
  if (d < 0){
    back = 1;
    d = -d;
  }
 
	get_robot_pose();

	double start_pos_x = robot_point_.x;
	double start_pos_y = robot_point_.y;
	double start_angle = robot_point_.z;

	ros::Rate loop_rate(freq_);

  int counter = 0;

	while(1){
		geometry_msgs::Twist vel;

    get_robot_pose();

    double now_angle = robot_point_.z;
    double ang_diff = start_angle - now_angle;
    double straight_diff = d - std::sqrt(std::pow(robot_point_.x - start_pos_x, 2.0) + std::pow(robot_point_.y - start_pos_y, 2.0)) ;

    std::cout << "ang diff:" << ang_diff;
    std::cout << " straight diff:" << straight_diff;

		vel.angular.z = ang_diff;
    if (vel.angular.z > 0.1)vel.angular.z = 0.1;
    if (vel.angular.z < -0.1)vel.angular.z = -0.1;

		vel.linear.x = straight_diff;
    if (back)vel.linear.x = - straight_diff;
    if (vel.linear.x > 0.1)vel.linear.x = 0.1;
    if (vel.linear.x < -0.1)vel.linear.x = -0.1;

    if(std::fabs(ang_diff) < 0.01 && std::fabs(straight_diff) < 0.005)counter++;
    else counter = 0;

    if(counter>40)break;

		std::cout << " count:" << counter << " straight:" << vel.linear.x << std::endl;

		velocity_pub_.publish(vel);
		loop_rate.sleep();
		ros::spinOnce();
	}
}

bool PanelAction::go_panel(double stop_distance){

	get_robot_pose();
	double start_angle = robot_point_.z;

	ros::Rate loop_rate(freq_);

  int counter = 0;
	while(scan_lock_)
  {
    std::cout << "no scan topic" << std::endl;
		loop_rate.sleep();
		ros::spinOnce();
  }
  
  static double pre_scan = 1000;
  while(1)
  {
    std::cout << "wait scan data stationary" << std::endl;
    std::cout << fabs(min_scan_ - pre_scan) << std::endl;
    if(fabs(min_scan_ - pre_scan) < 0.01)break;
    pre_scan = min_scan_;
		loop_rate.sleep();
		ros::spinOnce();
  }

	while(1){
		geometry_msgs::Twist vel;

    get_robot_pose();
    double now_angle = robot_point_.z;
    double ang_diff = now_angle - start_angle;
    std::cout << "ang diff:" << now_angle - start_angle;
		vel.angular.z = -ang_diff*3;
    if (vel.angular.z > 0.3)vel.angular.z = 0.3;
    if (vel.angular.z < -0.3)vel.angular.z = -0.3;

    if((min_scan_ > stop_distance - 0.01) && (min_scan_ < stop_distance + 0.01))counter++;
    else counter = 0;
    if(counter>40)break;
		vel.linear.x = min_scan_ - stop_distance;
    if (vel.linear.x > 0.3)vel.linear.x = 0.3;
    if (vel.linear.x < -0.3)vel.linear.x = -0.3;
		std::cout <<" min_scan:" << min_scan_ << " count:" << counter << " straight:" << vel.linear.x << std::endl;
		velocity_pub_.publish(vel);
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

bool PanelAction::up_arm(double height, double error_th){
  double now_height;
  int now_number = 1;
  const int max_number = 7;
  const int min_number = 1;
  elevator_navigation_srv::ArmMotion srv;
  srv.request.number = now_number;
  arm_motion_client_.call(srv);
  int loop = 1;
  
  while(loop){
    if (get_ar_marker_pose(now_height)){
      if (std::fabs(height - now_height) < error_th){
        ROS_INFO("finish");
        loop=0;
      }
      else if (height - now_height < 0){
        if (now_number < max_number)now_number++;
        srv.request.number = now_number;
        ROS_INFO("arm motion:%d", now_number);
        arm_motion_client_.call(srv);
      }
      else if (height - now_height > 0){
        if (now_number > min_number)now_number--;
        srv.request.number = now_number;
        ROS_INFO("arm motion:%d", now_number);
        arm_motion_client_.call(srv);
      }
      
    }
    else
    {
      ROS_ERROR("cannot get ar_marker pose");
    }
  }
}

bool PanelAction::home_arm(){
  elevator_navigation_srv::ArmMotion srv;
  srv.request.number = 0;
  ROS_INFO("arm motion:%d", 0);
  arm_motion_client_.call(srv);
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

bool PanelAction::get_ar_marker_pose(double &height){
        try
        {

                tf::StampedTransform trans;
                tf_listener_.waitForTransform(robot_frame_, ar_marker_frame_, ros::Time(0), ros::Duration(3.0));
                tf_listener_.lookupTransform(robot_frame_, ar_marker_frame_, ros::Time(0), trans);
                height = trans.getOrigin().z();
                return true;
        }
        catch(tf::TransformException &e)
        {
                ROS_WARN("%s", e.what());
                return false;
        }
}


int PanelAction::get_bounding_box_state(){
  return bounding_box_state_;
}

void PanelAction::boundingBoxCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  std::cout << "bounding box call back" << " state:" << msg->z << std::endl;
  bounding_box_x_ = msg -> x; 
  if(bounding_box_lock_)bounding_box_lock_--;
  
  bounding_box_state_= msg->z;
}


bool PanelAction::rotate_for_bounding_box(){
  const int bounding_box_target_x = 340;
  const int error_th = 5;
  const double max_ang_vel = 0.2;
  const int bounding_box_wait_count = 4;
  const int publish_vel_count = 3;
  int p_bounding_box_lock = 0;
  int orientation = 0;
  int break_state=0;
  ros::Rate r(10);
  ros::Time unchange_bounding_box_timer;
  int unchange_timer;
  geometry_msgs::Twist vel;
  while(1){
    int bounding_box_x_error = bounding_box_x_ - bounding_box_target_x;
    if (p_bounding_box_lock != bounding_box_lock_){
      unchange_bounding_box_timer = ros::Time::now();
    }
    unchange_timer = ros::Time::now().toSec() - unchange_bounding_box_timer.toSec();
    //std::cout << ros::Time::now().toSec() << " ";
    std::cout <<"lock:" << bounding_box_lock_ << "unchange time:" << unchange_timer << std::endl;
    if ((bounding_box_lock_== 0) || unchange_timer > 10)
    {
      unchange_bounding_box_timer = ros::Time::now();
      if (bounding_box_x_error < error_th && bounding_box_x_error > -error_th){
        break_state=1;
        break;
      }
      vel.angular.z = - bounding_box_x_error / 100.0;
      if (vel.angular.z > max_ang_vel){
        vel.angular.z = max_ang_vel;
      }
      else if (vel.angular.z < - max_ang_vel){
        vel.angular.z = -max_ang_vel;
      }
      bounding_box_lock_ = bounding_box_wait_count;
      p_bounding_box_lock = bounding_box_lock_;
    }

    if (vel.angular.z > 0)orientation++;
    else if(vel.angular.z < 0)orientation--;

    if (orientation>publish_vel_count){vel.angular.z=0;orientation=0;}
    else if (orientation< -publish_vel_count){vel.angular.z=0;orientation=0;}
    if(break_state)break;
    std::cout << "vel:" << vel.angular.z << "ori:" << orientation << "error:" << bounding_box_x_error << std::endl;
    velocity_pub_.publish(vel);
    r.sleep();
    ros::spinOnce();
  }
}

