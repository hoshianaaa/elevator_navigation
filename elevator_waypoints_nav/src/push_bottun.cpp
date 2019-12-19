#include <ros/ros.h>
#include <elevator_waypoints_nav/panel_action.h>
#include <elevator_navigation_srv/ArmMotion.h>
#include <elevator_navigation_srv/PushBottun.h>

bool push_bottun(elevator_navigation_srv::PushBottun::Request &req, elevator_navigation_srv::PushBottun::Response &res)
{
	PanelAction pa;

  //pa.rotate_for_bounding_box(req.bounding_box_x);
  //pa.go_panel(0.45);
  //pa.go_panel(req.stop_distance);

  int loop = 1;
  while(loop){
    pa.up_arm(1.18, 0.005);
    /*
    //pa.rotate_for_bounding_box(330);
    pa.rotate_for_bounding_box(req.bounding_box_x);
    pa.go_panel(req.stop_distance);
    pa.up_arm(req.height, 0.005);
    //pa.straight(0.08);
    pa.straight(req.go_distance);
    pa.straight(-req.go_distance);
    pa.home_arm();
    if (pa.find_bounding_box(3))std::cout << "find bounding box 3" << std::endl;
    */
  }
  res.status = true;
  return true;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "panel_action_node");
	ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("push_bottun", push_bottun);
  ros::spin();
	return (0);
}
