#include <elevator_waypoints_nav/panel_action.h>

PanelAction::PanelAction()
{
	std::cout << "panel action class" << std::endl;
	velocity_pub_ = n_.advertise<geometry_msgs::Twist>("icart_mini/cmd_vel", 1);
	darknet_sub_ = n_.subscribe("darknet_data", 1, &PanelAction::darknetCallback, this);
}

void PanelAction::run()
{

}

void PanelAction::darknetCallback(const std_msgs::Int32Ptr& msg){
	std::cout << "darknet callback" << std::endl;
}
