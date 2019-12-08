#include <ros/ros.h>
#include <elevator_waypoints_nav/panel_action.h>
#include <elevator_navigation_srv/ArmMotion.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "panel_action_node");
	ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<elevator_navigation_srv::ArmMotion>("arm_motion");
  elevator_navigation_srv::ArmMotion srv;

	PanelAction pa;
  while(1){
    std::cout << "bottun state:" << pa.get_bounding_box_state() << std::endl;
    ros::spinOnce();
    //pa.rotate_for_bounding_box();
  }
  pa.go_panel(0.45);

  int loop = 1;
  while(loop){
    pa.rotate_for_bounding_box();
    pa.go_panel(0.45);
    pa.up_arm(1.28, 0.005);
    pa.straight(0.04);
    pa.straight(-0.04);
    pa.home_arm();
    std::cout << "bottun state:" << pa.get_bounding_box_state() << std::endl;
    if (pa.get_bounding_box_state() == 1)loop = 0;
  }
	return (0);
}
