#include <elevator_waypoints_nav/action.h>

Action::Action(geometry_msgs::Point sp, geometry_msgs::Point fp, bool rotate)
{
	velocity_pub_ = n_.advertise<geometry_msgs::Twist>("icart_mini/cmd_vel", 1);
	scan_sub_ = n_.subscribe("scan", 1, &Action::scanCallback,this); 

	start_point_ = sp;
	goal_point_ = fp;

	goal_angle_ = calc_goal_angle(sp,fp,rotate);

	robot_frame_ = "base_link";
	global_frame_ = "map";

	speed_ = 0.5;
}

bool Action::get_robot_pose(){
	try
	{
		tf::StampedTransform trans;
		tf_listener_.waitForTransform(global_frame_, robot_frame_, ros::Time(0), ros::Duration(3.0));
		tf_listener_.lookupTransform(global_frame_, robot_frame_, ros::Time(0), trans);
		robot_x_ = trans.getOrigin().x();
		robot_y_ = trans.getOrigin().y();
		robot_yaw_ = tf::getYaw(trans.getRotation());
		return true;
	}
	catch(tf::TransformException &e)
	{
		ROS_WARN("%s", e.what());
		return false;
	}
}

void Action::move(){
	double ang_speed=0;
	double line_distance_gain = 0.1;
	double angToline_gain = 0.1;
	while(!on_goal()){
		while(object()){
			publish_vel(0,0);
		}
		geometry_msgs::Point sp,fp,rp;
		ang_speed = line_distance_gain * calc_distance_line(sp, fp, rp) + calc_angToline(sp, fp, rp);
		publish_vel(speed_, ang_speed);
	}
	if (rotate_on_goal_)rotate();
}

void Action::rotate(){

}

bool Action::object(){
	return 0;
}

bool Action::on_goal(){
	return 0;
}
		
void Action::publish_vel(double speed, double ang_speed){
	geometry_msgs::Twist vel;
	vel.linear.x = speed;
	vel.angular.z = ang_speed;
	velocity_pub_.publish(vel);
}

double Action::calc_goal_angle(geometry_msgs::Point start_point, geometry_msgs::Point finish_point, bool rotate){
	return 0;
}

double Action::calc_distance_line(geometry_msgs::Point start_point, geometry_msgs::Point finish_point, geometry_msgs::Point robot_point){
	return 0;
}

double Action::calc_angToline(geometry_msgs::Point start_point, geometry_msgs::Point finish_point, geometry_msgs::Point robot_point){
	return 0;
}

void Action::scanCallback(const sensor_msgs::LaserScanPtr& msg){
}

void Action::print_env_data(){
	std::cout << "start_point = " << "(" << start_point_.x << ", " << start_point_.y << ", " << start_point_.z << ")" <<  std::endl;
	std::cout << "goal_point = " << "(" << goal_point_.x << ", " << goal_point_.y << ", " << goal_point_.z << ")" <<  std::endl;
	std::cout << "goal_angle:" << goal_angle_ << std::endl;
}

void Action::print_robot_data(){
	std::cout << "  x:" << robot_x_ << std::endl;
	std::cout << "  y:" << robot_y_ << std::endl;
	std::cout << "yaw:" << robot_yaw_ << std::endl;
}
