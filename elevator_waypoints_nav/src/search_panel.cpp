#include <elevator_waypoints_nav/search_panel.h>

SearchPanel::SearchPanel()
{
	std::cout << "panel action class" << std::endl;
	velocity_pub_ = n_.advertise<geometry_msgs::Twist>("icart_mini/cmd_vel", 1);
	darknet_sub_ = n_.subscribe("darknet_data", 1, &SearchPanel::darknetCallback, this);
	darknet_lock_ = 0;
	robot_frame_  = "base_link";
	global_frame_ = "odom";
}



void SearchPanel::darknetCallback(const std_msgs::Int32Ptr& msg){
	std::cout << "darknet callback" << std::endl;
	darknet_lock_ = 0;
}

bool SearchPanel::rotate()
{
while(1){
	std::cout << "search panel rotate" << std::endl;
}

}

bool SearchPanel::darknet_wait(){
	darknet_lock_ = 1;
	while(darknet_lock_){
		std::cout << "darknet lock" << std::endl;
	}
	return 1;
}

void SearchPanel::run()
{

}

bool SearchPanel::get_robot_pose(){
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

