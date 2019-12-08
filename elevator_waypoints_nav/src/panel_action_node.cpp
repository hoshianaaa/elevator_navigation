#include <ros/ros.h>
#include <elevator_waypoints_nav/panel_action.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

int bottun_x_error = 100;
double panel_distance = 100;
int bounding_box_lock = 3;



int main(int argc, char** argv){
	ros::init(argc, argv, "panel_action_node");
	ros::NodeHandle n;
	PanelAction pa;
	pa.rotate(M_PI/8);
	pa.go_panel(0.45);
  pa.rotate_for_bounding_box();
  pa.go_panel(0.45);

  int loop = 1;
  while(loop){
    pa.rotate_for_bounding_box();
    pa.go_panel(0.45);
    pa.up_arm(1.275, 0.005);
    pa.straight(0.04);
    pa.straight(-0.04);
    pa.home_arm();
    std::cout << "bottun state:" << pa.get_bounding_box_state() << std::endl;
    if (pa.get_bounding_box_state() == 1)loop = 0;
  }
	return (0);
}
