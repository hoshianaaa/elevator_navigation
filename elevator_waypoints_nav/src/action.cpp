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
	ang_speed_max_ = 0.5;
}

bool Action::get_robot_pose(){
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

void Action::move(){
	double ang_speed=0;
	double line_distance_gain = 10;
	double angToline_gain = 10;
	get_robot_pose();
	while(!on_goal()){
		get_robot_pose();
		while(object()){
			publish_vel(0,0);
		}
		geometry_msgs::Point sp,fp,rp;
		sp = start_point_;
		fp = goal_point_;
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
	if(ang_speed > ang_speed_max_)ang_speed = ang_speed_max_;
	else if(ang_speed < -ang_speed_max_)ang_speed = -ang_speed_max_;
	vel.angular.z = ang_speed;
	velocity_pub_.publish(vel);
}

double Action::calc_goal_angle(geometry_msgs::Point start_point, geometry_msgs::Point finish_point, bool rotate){
	double a1, a2;

	if (rotate > 0){
		a1 = -(finish_point.x - start_point.x);
		a2 = -(finish_point.y - start_point.y);
	}
	else{
		a1 = (finish_point.x - start_point.x);
		a2 = (finish_point.y - start_point.y);
	}


	return std::atan2(a2, a1);
}

double Action::calc_distance_line(geometry_msgs::Point start_point, geometry_msgs::Point finish_point, geometry_msgs::Point robot_point){
	//vector a[a1, a2]:sp->fp
	//vector b[b1, b2]:sp->rp
	
	double a1, a2, b1, b2, in_ang;

	a1 = finish_point.x - start_point.x;
	a2 = finish_point.y - start_point.y;

	b1 = robot_point.x - start_point.x;
	b2 = robot_point.y - start_point.y;

	double len_b = std::sqrt(b1*b1+b2*b2);	
	
	bool sign = false;// + or -
	in_ang = calc_vectors_interior_angle(a1, a2, b1, b2);
	if (in_ang > 0)sign = true;
	else sign = false;
	
	double c = len_b*std::cos(in_ang);
	double distance = std::sqrt(len_b*len_b - c*c);

	if (sign == true)return distance;
	else return -distance;
}

double Action::calc_angToline(geometry_msgs::Point start_point, geometry_msgs::Point finish_point, geometry_msgs::Point robot_point){
	//vector a[a1, a2]:fp->sp
	//vector b[b1, b2]:fp->rp
	
	double a1, a2, b1, b2, in_ang;

	a1 = start_point.x - finish_point.x;
	a2 = start_point.y - finish_point.y;

	b1 = robot_point.x - finish_point.x;
	b2 = robot_point.y - finish_point.y;
	
	in_ang = calc_vectors_interior_angle(a1, a2, b1, b2);
	return in_ang;
}

double Action::calc_vectors_interior_angle(const double a1, const double a2, const double b1, const double b2){

	double len_a = std::sqrt(a1*a1 + a2*a2);
	double len_b = std::sqrt(b1*b1 + b2*b2);
	
	double inner_product = a1*b1 + a2*b2;
	double ang = std::acos(inner_product / (len_a * len_b));

	double outer_product1 = a2*0 - 0*b2;
	double outer_product2 = 0*b1 - a1*0;
	double outer_product3 = a1*b2 - a2*b1;
	
	if (outer_product3 < 0)return -ang;
	else return ang;
}

void Action::scanCallback(const sensor_msgs::LaserScanPtr& msg){
}

void Action::print_env_data(){
	std::cout << "start_point = " << "(" << start_point_.x << ", " << start_point_.y << ", " << start_point_.z << ")" <<  std::endl;
	std::cout << "goal_point = " << "(" << goal_point_.x << ", " << goal_point_.y << ", " << goal_point_.z << ")" <<  std::endl;
	std::cout << "goal_angle:" << goal_angle_ << std::endl;
}

void Action::print_robot_data(){
	std::cout << "  x:" << robot_point_.x << std::endl;
	std::cout << "  y:" << robot_point_.y << std::endl;
	std::cout << "yaw:" << robot_point_.z << std::endl;
}
