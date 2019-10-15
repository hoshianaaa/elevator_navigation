#include <elevator_waypoints_nav/action.h>

Action::Action(geometry_msgs::Point sp, geometry_msgs::Point fp, bool rotate)
{
	velocity_pub_ = n_.advertise<geometry_msgs::Twist>("icart_mini/cmd_vel", 1);
	scan_sub_ = n_.subscribe("scan", 1, &Action::scanCallback,this); 

	start_point_ = sp;
	goal_point_ = fp;

	if(!calc_goal_angle(sp,fp,rotate,goal_angle_)){
		ROS_ERROR("cannot calc goal_angle");
	}

	robot_frame_ = "base_link";
	global_frame_ = "map";

	speed_ = 0.3;
	ang_speed_max_ = 0.5;

	freq_ = 10;
	sensor_range_ = M_PI / 180 * 5;
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
	double ang_speed =0.0;

	double gain_to_dis_diff = -2.0;
	double gain_to_ang_diff = 2.0;
	double gain_to_pose_diff = -2.0;

	ros::Rate loop_rate(freq_);
	get_robot_pose();
	//loop_rate.sleep();
	while(!on_goal()){
		std::cout << "debug" << StopRobot() << std::endl;
		get_robot_pose();
		while(StopRobot()){
			publish_vel(0,0);
			loop_rate.sleep();
		}
		geometry_msgs::Point sp,fp,rp;
		sp = start_point_;
		fp = goal_point_;

		get_robot_pose();
		rp = robot_point_;
		double dis_diff, ang_diff, pose_diff;
		if (calc_dis_diff(sp, fp, rp, dis_diff)){
			if (calc_ang_diff(sp, fp, rp, ang_diff)){
				if (calc_pose_diff(rp.z, pose_diff)){
					std::cout << "dis:" << dis_diff << " ang:" << ang_diff << " pose:" << pose_diff <<  std::endl;
					print_env_data();
					ang_speed = gain_to_dis_diff * dis_diff + gain_to_ang_diff * ang_diff + gain_to_pose_diff * pose_diff;
					publish_vel(speed_, ang_speed);
					loop_rate.sleep();
				}
			}
		}
		loop_rate.sleep();
	}
	if(rotate(0.5, 0.2))std::cout << "finish rotate 1" << std::endl;
	if(rotate(0.2, 0.05))std::cout << "finish rotate 2" << std::endl;
}

bool Action::rotate(double speed, double goal_angle_th){
	ros::Rate loop_rate(freq_);
	double ang_speed, ang_diff;
	double gain_to_ang_diff = -0.2;
	ang_diff = 10000;
	get_robot_pose();

	double a[2], b[2];
	a[0] = std::cos(goal_angle_);
	a[1] = std::sin(goal_angle_);

	b[0] = std::cos(robot_point_.z);
	b[1] = std::sin(robot_point_.z);

	calc_vectors_interior_angle(a[0], a[1], b[0], b[1], ang_diff);

	while(std::abs(ang_diff) > goal_angle_th){
		get_robot_pose();
		ang_diff = robot_point_.z - goal_angle_;
		if (ang_diff > 0)ang_speed = -speed;
		else ang_speed = speed;
		publish_vel(0, ang_speed);
		loop_rate.sleep();
	}
	return true;
}

bool Action::StopRobot(){
	if (sensor_data_min_ < 0.6){
		std::cout << "stop!!" << sensor_data_min_ <<  std::endl;
		return 1;
	}
	return 0;
}

bool Action::on_goal(){

	double goal_distance_th, dx, dy, dis;

	goal_distance_th = 0.2;
	get_robot_pose();

	dx = robot_point_.x - goal_point_.x;
	dy = robot_point_.y - goal_point_.y;

	dis = std::sqrt(dx*dx + dy*dy);

	std::cout << "goal--robot dis:" << dis << std::endl;

	if (dis < goal_distance_th){
		return true;
	}
	return false;
}

	void Action::publish_vel(double speed, double ang_speed){
	geometry_msgs::Twist vel;
	vel.linear.x = speed;
	if(ang_speed > ang_speed_max_)ang_speed = ang_speed_max_;
	else if(ang_speed < -ang_speed_max_)ang_speed = -ang_speed_max_;
	vel.angular.z = ang_speed;

	std::cout << "vel:" << vel.linear.x << " " << vel.angular.z << std::endl;
	velocity_pub_.publish(vel);
}

bool Action::calc_goal_angle(const geometry_msgs::Point start_point, const geometry_msgs::Point finish_point, const bool rotate, double& ang){
	double a1, a2;

	if (rotate > 0){
		a1 = -(finish_point.x - start_point.x);
		a2 = -(finish_point.y - start_point.y);
	}
	else{
		a1 = (finish_point.x - start_point.x);
		a2 = (finish_point.y - start_point.y);
	}

	ang = std::atan2(a2, a1);
	return true;
}

bool Action::calc_dis_diff(const geometry_msgs::Point start_point, const geometry_msgs::Point finish_point, const geometry_msgs::Point robot_point, double& distance){
	//vector a[a1, a2]:sp->fp
	//vector b[b1, b2]:sp->rp
	
	double a1, a2, b1, b2, in_ang;

	a1 = finish_point.x - start_point.x;
	a2 = finish_point.y - start_point.y;

	b1 = robot_point.x - start_point.x;
	b2 = robot_point.y - start_point.y;

	double len_b = std::sqrt(b1*b1+b2*b2);	
	
	bool sign = false;// + or -
	if(!calc_vectors_interior_angle(a1, a2, b1, b2, in_ang))return false;
	if (in_ang > 0)sign = true;
	else sign = false;
	
	double c = len_b*std::cos(in_ang);
	distance = std::sqrt(len_b*len_b - c*c);

	if (sign != true){
		distance = -distance;
	}
	return true;
}

bool Action::calc_ang_diff(const geometry_msgs::Point start_point, const geometry_msgs::Point finish_point, const geometry_msgs::Point robot_point, double& in_ang){
	//vector a[a1, a2]:fp->sp
	//vector b[b1, b2]:fp->rp
	
	double a1, a2, b1, b2;

	a1 = start_point.x - finish_point.x;
	a2 = start_point.y - finish_point.y;

	b1 = robot_point.x - finish_point.x;
	b2 = robot_point.y - finish_point.y;
	
	if(calc_vectors_interior_angle(a1, a2, b1, b2, in_ang)){
		return true;
	}
	else{
		return false;
	}
}

bool Action::calc_vectors_interior_angle(const double a1, const double a2, const double b1, const double b2, double& ang){

	double len_a = std::sqrt(a1*a1 + a2*a2);
	double len_b = std::sqrt(b1*b1 + b2*b2);

	if (len_a == 0 || len_b == 0){
		ang = M_PI / 2;
		return true;
	}
	
	double inner_product = a1*b1 + a2*b2;
	ang = std::acos(inner_product / (len_a * len_b));

	double outer_product1 = a2*0 - 0*b2;
	double outer_product2 = 0*b1 - a1*0;
	double outer_product3 = a1*b2 - a2*b1;
	
	if (outer_product3 < 0)ang = -ang;
	return true;
}

bool Action::calc_pose_diff(const double robot_yaw, double& ang){

	double a1, a2, b1, b2;
	a1 = goal_point_.x - start_point_.x;
	a2 = goal_point_.y - start_point_.y;
	b1 = std::cos(robot_yaw);
	b2 = std::sin(robot_yaw);

	if(calc_vectors_interior_angle(a1, a2, b1, b2, ang))return true;
	return false;
}

class sensor_data{
	public:
		double data;
		double angle;
};

void Action::scanCallback(const sensor_msgs::LaserScanPtr& msg){
	std::cout << "scan call back" << std::endl;
	int num = (msg->angle_max - msg->angle_min) / msg->angle_increment;

	std::vector<sensor_data> datas;
	for (int i=0;i<num;i++){
		sensor_data d;
		d.data = msg->ranges[i];
		d.angle = msg->angle_min + i * msg->angle_increment;
		if (d.angle < sensor_range_ && d.angle > -sensor_range_){
			datas.push_back(d);
		}
	}

	double min = 10000;
	for (int i=0;i < datas.size();i++){
		if (datas[i].data > 0.1)min = std::min(datas[i].data, min);
	}

	sensor_data_min_ = min;
	std::cout << "sensor data mini:" <<  sensor_data_min_ << std::endl;
}

void Action::print_env_data(){
	std::cout << "start_point = " << "(" << start_point_.x << ", " << start_point_.y << ", " << start_point_.z << ")" <<  std::endl;
	std::cout << "goal_point = " << "(" << goal_point_.x << ", " << goal_point_.y << ", " << goal_point_.z << ")" <<  std::endl;
	std::cout << "goal_angle:" << goal_angle_ << std::endl;
	std::cout << "sensor_data_min:" << sensor_data_min_ << std::endl;
}

void Action::print_robot_data(){
	std::cout << "  x:" << robot_point_.x << std::endl;
	std::cout << "  y:" << robot_point_.y << std::endl;
	std::cout << "yaw:" << robot_point_.z << std::endl;
}
