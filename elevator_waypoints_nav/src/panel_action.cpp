#include <elevator_waypoints_nav/panel_action.h>
const int STOP_TRACK_B_BOX = 3333333;

PanelAction::PanelAction(std::string name)
{
	//std::cout << "panel action class" << std::endl;
	velocity_pub_ = n_.advertise<geometry_msgs::Twist>("icart_mini/cmd_vel", 1);
	scan_sub_ = n_.subscribe("scan", 1, &PanelAction::scanCallback, this);
  
  std::string topic_name = name + "bounding_box_pos";
	bounding_box_sub_ = n_.subscribe(topic_name, 1, &PanelAction::boundingBoxCallback, this);
  arm_motion_client_ = n_.serviceClient<elevator_navigation_srv::ArmMotion>("arm_motion");
  arm_motion_down_client_ = n_.serviceClient<elevator_navigation_srv::ArmMotionDown>("arm_motion_down");
	
	scan_lock_ = 1;
	robot_frame_  = "base_link";
	global_frame_ = "odom";
  ar_marker_frame_ = "ar_marker_187";
	freq_ = 10;
	min_scan_ = 0;

	get_robot_pose();
	home_point_ = robot_point_;

  bounding_box_lock_ = 4;

  track_b_box_id_ = STOP_TRACK_B_BOX;
  found_b_box_ = 0;
}

void PanelAction::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  scan_lock_ = 0;
  min_scan_ = min_scan_ * 0.7 + msg->ranges[msg->ranges.size()/2] * 0.3;
  scan_sum_ = 0;
  for(int i=0;i<msg->ranges.size();i++){
    scan_sum_ += msg->ranges[i];
  }
}

bool PanelAction::rotate_for_target_angle(double target_angle)
{
	ros::Rate loop_rate(freq_);
  const double limit_ang_vel = 0.3;
  const double error_th = M_PI/180;
  int counter = 0;
  geometry_msgs::Twist vel;

  fix_angle(target_angle);

	while(1){

    get_robot_pose();

    if((robot_point_.z > target_angle - error_th) && (robot_point_.z < target_angle + error_th))counter++;
    else counter = 0;
    if(counter>5)break;

    double diff = target_angle - robot_point_.z;
    fix_angle(diff);

    vel.angular.z = diff;
    if(vel.angular.z > limit_ang_vel)vel.angular.z = limit_ang_vel;
    if(vel.angular.z < -limit_ang_vel)vel.angular.z = -limit_ang_vel;
		velocity_pub_.publish(vel);

		//std::cout << "target:" << target_angle/M_PI*180 << " now:" << robot_point_.z/M_PI*180 << " ang_vel:" << vel.angular.z << std::endl;	
		loop_rate.sleep();
		ros::spinOnce();
  }
}

bool PanelAction::rotate_home()
{
  rotate_for_target_angle(home_point_.z);
}



bool PanelAction::rotate(double angle)
{
	ros::Rate loop_rate(freq_);
	get_robot_pose();
	double start_angle = robot_point_.z;
	double target_angle = start_angle + angle; 
  
  rotate_for_target_angle(target_angle);
}

bool PanelAction::fix_angle(double& angle){
  if (angle > M_PI)angle -= 2*M_PI;
  if (angle < -M_PI)angle += 2*M_PI;
}

bool PanelAction::line_tracking_stop_point(double x, double y, double angle){

  const double vel_max = 0.3;
  const double ang_vel_max = 0.6;
  const double dis_bias = 0.5;
  const double ang_bias = 0.5;
  double line_v[2] = {std::cos(angle), std::sin(angle)};
  double len_line_v = calc_distance(line_v[0], line_v[1]);
	ros::Rate loop_rate(freq_);
  geometry_msgs::Twist vel;

	while(1){

    get_robot_pose();
    double robot_v[2] = {robot_point_.x-x, robot_point_.y-y};
    //std::cout << "rx:" << robot_v[0] << " ry:" << robot_v[1] << std::endl;
    double inner_pro = calc_inner_product(line_v[0], line_v[1], robot_v[0], robot_v[1]);
    double len_contact_point_v = inner_pro / len_line_v;

    double contact_point_v[2] = {len_contact_point_v/len_line_v*std::cos(angle), len_contact_point_v/len_line_v*std::sin(angle)};
    //std::cout << "cx:" << contact_point_v[0] << " cy:" << contact_point_v[1] << std::endl;

    double perpendicular_line_v[2] = {robot_v[0] - contact_point_v[0], robot_v[1] - contact_point_v[1]};
    double diff = calc_distance(perpendicular_line_v[0], perpendicular_line_v[1]);
    double outer_product_z = line_v[0]*perpendicular_line_v[1] - perpendicular_line_v[0]*line_v[1];

    if (outer_product_z < 0)diff = -diff;
    //std::cout << "diff:" << diff << std::endl;

    double ang_diff = angle - robot_point_.z;
    fix_angle(ang_diff);

    vel.angular.z = ang_bias*ang_diff - dis_bias*diff;
    if(vel.angular.z > ang_vel_max)vel.angular.z = ang_vel_max;
    if(vel.angular.z < -ang_vel_max)vel.angular.z = -ang_vel_max;

    double len_robot_v = calc_distance(robot_v[0], robot_v[1]);

    double goal_dis = len_robot_v;
    vel.linear.x = goal_dis * (1 - std::fabs(vel.angular.z)/ang_vel_max);
    if(vel.linear.x > vel_max)vel.linear.x = vel_max;
    if(vel.linear.x < -vel_max)vel.linear.x = -vel_max;

		velocity_pub_.publish(vel);

		//std::cout << "diff:" << diff << " ang_diff:" << ang_diff << " goal_dis:" << goal_dis << std::endl;
		//std::cout << "vel:" << vel.linear.x << " ang_vel:" << vel.angular.z << std::endl;
		loop_rate.sleep();
		ros::spinOnce();
  }
}

double PanelAction::calc_distance(double x, double y){
  return std::sqrt(std::fabs(x) + std::fabs(y));
}

double PanelAction::calc_inner_product(double x1, double y1, double x2, double y2){
  return x1*x2 + y1*y2;
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

    //std::cout << "ang diff:" << ang_diff;
    //std::cout << " straight diff:" << straight_diff;

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

		//std::cout << " count:" << counter << " straight:" << vel.linear.x << std::endl;

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
    //std::cout << "no scan topic" << std::endl;
		loop_rate.sleep();
		ros::spinOnce();
  }
  
  static double pre_scan = 1000;
  while(1)
  {
    //std::cout << "wait scan data stationary" << std::endl;
    //std::cout << fabs(min_scan_ - pre_scan) << std::endl;
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
    //std::cout << "ang diff:" << now_angle - start_angle;
		vel.angular.z = -ang_diff*3;
    if (vel.angular.z > 0.3)vel.angular.z = 0.3;
    if (vel.angular.z < -0.3)vel.angular.z = -0.3;

    if((min_scan_ > stop_distance - 0.01) && (min_scan_ < stop_distance + 0.01))counter++;
    else counter = 0;
    if(counter>40)break;
		vel.linear.x = min_scan_ - stop_distance;
    if (vel.linear.x > 0.3)vel.linear.x = 0.3;
    if (vel.linear.x < -0.3)vel.linear.x = -0.3;
		//std::cout <<" min_scan:" << min_scan_ << " count:" << counter << " straight:" << vel.linear.x << std::endl;
		velocity_pub_.publish(vel);
		loop_rate.sleep();
		ros::spinOnce();
	}
}

bool PanelAction::back(double distance, double speed){
  //std::cout << "back" << std::endl;
	ros::Rate loop_rate(freq_);
	double stop_distance = 0.1;
  get_robot_pose();
  
  double sx = robot_point_.x;
  double sy = robot_point_.y;

  geometry_msgs::Twist vel;
	while(1){

    get_robot_pose();

		double dx = robot_point_.x - sx;
		double dy = robot_point_.y - sy;

		double now_dis = std::sqrt(dx * dx + dy * dy);
		if (now_dis > distance)break;
		vel.linear.x = -speed;
		velocity_pub_.publish(vel);
		loop_rate.sleep();
		ros::spinOnce();

	}
		vel.linear.x = 0.0;
		velocity_pub_.publish(vel);
	
}

bool PanelAction::up_arm(double height, double error_th, int start_number){
  double now_height;
  int now_number = start_number;
  const int max_number = 17;
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

bool PanelAction::down_arm(double height, double error_th, int start_number){
  double now_height;
  int now_number = start_number;
  const int max_number = 5;
  const int min_number = 1;
  elevator_navigation_srv::ArmMotionDown srv;
  srv.request.number = now_number;
  arm_motion_down_client_.call(srv);
  int loop = 1;
  
  while(loop){
    if (get_ar_marker_pose(now_height)){
      std::cout << "now height:" << now_height << " height:" << height << std::endl;
      if (std::fabs(height - now_height) < error_th){
        ROS_INFO("finish");
        loop=0;
      }
      else if (height - now_height < 0){
        if (now_number < max_number)now_number++;
        srv.request.number = now_number;
        ROS_INFO("arm motion down (down):%d", now_number);
        arm_motion_down_client_.call(srv);
      }
      else if (height - now_height > 0){
        if (now_number > min_number)now_number--;
        srv.request.number = now_number;
        ROS_INFO("arm motion down (up):%d", now_number);
        arm_motion_down_client_.call(srv);
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

bool PanelAction::home_arm_down(){
  elevator_navigation_srv::ArmMotionDown srv;
  srv.request.number = 0;
  ROS_INFO("arm motion:%d", 0);
  arm_motion_down_client_.call(srv);
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


bool PanelAction::find_bounding_box(const int id){
  ros::spinOnce();
  for(int i=0;i<boxes_.size();i++){
    if (boxes_[i].id == id)return 1;
  }
  return 0;
}

void PanelAction::boundingBoxCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  //std::cout << "bounding box call back" << " state:" << msg->z << std::endl;

  BoundingBox b;
  boxes_.clear();

  for(int i=0;i<msg->poses.size();i++){
    b.x = msg ->poses[i].position.x;
    b.id = msg ->poses[i].position.z;
    std::cout << "debug0 track_b_box_id:"<< track_b_box_id_ << std::endl;
    if(track_b_box_id_ != STOP_TRACK_B_BOX){
      std::cout << "debug1 !!!!!" << std::endl;
      if(b.id == track_b_box_id_){
        found_b_box_ = 1;
        std::cout << "debug2 !!!!! found_b_box:" << found_b_box_ << std::endl;
      }
    }
    boxes_.push_back(b);
  }

  if(bounding_box_lock_)bounding_box_lock_--;

 
}


//id up:0 up_on:1 e18:2 e18_on:3
bool PanelAction::rotate_for_bounding_box(const int bounding_box_target_x, const int target_id, const int error_th){
  const double max_ang_vel = 0.2;
  const int bounding_box_wait_count = 4;
  const int publish_vel_count = 3;
  int p_bounding_box_lock = 0;
  int orientation = 0;
  int break_state=0;
  ros::Rate r(10);
  ros::Time unchange_bounding_box_timer;
  long unchange_timer;
  geometry_msgs::Twist vel;
  int bounding_box_x_error = 10000;
  while(1){
    for(int i=0;i<boxes_.size();i++){
      if(boxes_[i].id == target_id)bounding_box_x_error = boxes_[i].x - bounding_box_target_x;
    }
    if (p_bounding_box_lock != bounding_box_lock_){
      unchange_bounding_box_timer = ros::Time::now();
    }
    unchange_timer = ros::Time::now().toSec() - unchange_bounding_box_timer.toSec();
    //std::cout <<"lock:" << bounding_box_lock_ << "unchange time:" << unchange_timer << std::endl;
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
    }

    p_bounding_box_lock = bounding_box_lock_;

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

double PanelAction::get_scan_sum(){
  return scan_sum_;
}

void PanelAction::track_b_box_start(int id){
  track_b_box_id_ = id;
}

void PanelAction::track_b_box_clear(){
  track_b_box_id_ = 333;
  found_b_box_ = 0;
}

bool PanelAction::get_found_b_box_state(){
  ros::Rate r(10);
  r.sleep();
  ros::spinOnce();
  return found_b_box_;
}
