#include <ros/ros.h>
#include <elevator_waypoints_nav/panel_action.h>
#include <elevator_navigation_srv/ArmMotion.h>
#include <elevator_navigation_srv/PushBottun.h>

bool push_bottun(elevator_navigation_srv::PushBottun::Request &req, elevator_navigation_srv::PushBottun::Response &res)
{
  std::cout << "push bottun debug1" << std::endl;
	PanelAction pa("up");

  //pa.rotate_for_bounding_box(req.bounding_box_x);
  //pa.go_panel(0.45);
  //pa.go_panel(req.stop_distance);

  int loop = 1;
  std::cout << "push bottun debug2" << std::endl;

  std::cout << "stop action:" << pa.stop_action() << std::endl;
  pa.track_b_box_start(1);
  pa.check_door_start();
  ros::Rate r(1);
  while(loop){
    std::cout << "stop action:" << pa.stop_action() << std::endl;
    //std::cout << "rotate found:" << pa.rotate_for_bounding_box(req.bounding_box_x, 1, 5) << std::endl;
    //pa.home_arm();
    //std::cout << "scan sum:" << pa.get_scan_sum() << std::endl;
    //pa.up_arm(1.18, 0.005);
    /*
    //pa.rotate_for_bounding_box(330);
    pa.go_panel(req.stop_distance);
    pa.up_arm(req.height, 0.005);
    //pa.straight(0.08);
    pa.straight(req.go_distance);
    pa.straight(-req.go_distance);
    if (pa.find_bounding_box(3))std::cout << "find bounding box 3" << std::endl;
    */
    r.sleep();
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
