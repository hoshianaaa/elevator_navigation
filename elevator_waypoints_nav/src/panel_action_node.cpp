#include <ros/ros.h>
#include <elevator_waypoints_nav/panel_action.h>
#include <elevator_waypoints_nav/action.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <elevator_navigation_srv/ElevatorAction.h>
#include <elevator_navigation_srv/PushBottun.h>

int bottun_x_error = 100;
double panel_distance = 100;
int bounding_box_lock = 3;

ros::ServiceClient push_bottun_client_;

bool elevator_action(elevator_navigation_srv::ElevatorAction::Request &req, elevator_navigation_srv::ElevatorAction::Response &res)
{
	PanelAction pa;
  //in elevator
	pa.rotate(M_PI/8);
	pa.go_panel(0.45);
  pa.rotate_for_bounding_box(330);
  pa.go_panel(0.45);
  
  int bounding_box_x = 340;
  int bounding_box_stop_number = 1;
  //double height = 1.275;
  double height = 1.2;
  double stop_distance = 0.45;
  double go_distance = 0.05;

  double first_scan_sum, scan_sum;
  first_scan_sum = pa.get_scan_sum();
  std::cout << std::endl << std::endl <<  "first scan sum!!!!!!!:" << first_scan_sum << std::endl;

  int id = 0;
  while(1){

    pa.rotate_for_bounding_box(bounding_box_x);
    pa.go_panel(stop_distance);
    pa.up_arm(height, 0.005);
    
    pa.straight(go_distance);

    scan_sum = pa.get_scan_sum();
    std::cout << "scan sum!!!!!!!!!:" << scan_sum << std::endl;
    if((scan_sum - first_scan_sum) > 60){
      std::cout << "open the door!!!!!!!!!!"  << std::endl;
      pa.back(0.5, 0.5);
      id = 2;
      break;
    }

    pa.straight(-go_distance);


    pa.home_arm();
    std::cout << "bottun state:" << pa.get_bounding_box_state() << std::endl;
    if (pa.get_bounding_box_state() == bounding_box_stop_number){
      id =1;
      break;
    }

  }

  geometry_msgs::Point start_point, goal_point;
  if(id == 1){
    start_point.x = 2;
    start_point.y = -0.7;
    goal_point.x = 2;
    goal_point.y = 0;
    Action go_to_elevator(start_point, goal_point, false);

    go_to_elevator.rotate(0.3, 0.3);
    go_to_elevator.move();

    start_point.x = 2;
    start_point.y = 0;
    goal_point.x = 1.8;
    goal_point.y = 0;
    Action near_elevator(start_point, goal_point, false, false);

    near_elevator.rotate(0.3, 0.3);
    near_elevator.move();
  }

 	start_point.x = 1.8;
	start_point.y = 0;
	goal_point.x = 0;
	goal_point.y = -0.3;
  if(id == 1){
    Action in_elevator(start_point, goal_point, true);
    in_elevator.move();
  }
  if(id == 2){
    Action in_elevator(start_point, goal_point, true, false);
    in_elevator.move();
    pa.home_arm();
  }

  //out elevator
	PanelAction pa2;
	pa2.rotate(-M_PI/6);
	pa2.go_panel(0.45);
  pa2.rotate_for_bounding_box(340);
  pa2.go_panel(0.45);

  bounding_box_x = 330;
  bounding_box_stop_number = 3;
  height = 1.30;
  stop_distance = 0.45;
  go_distance = 0.09;

  while(1){
    pa2.rotate_for_bounding_box(bounding_box_x);
    pa2.go_panel(stop_distance);
    pa2.up_arm(height, 0.005);
    pa2.straight(go_distance);
    pa2.straight(-go_distance);
    pa2.home_arm();
    std::cout << "bottun state:" << pa2.get_bounding_box_state() << std::endl;
    if (pa2.get_bounding_box_state() == bounding_box_stop_number)break;
  }

  pa2.back(0.50);
  pa2.rotate_home();

 	start_point.x = 0;
	start_point.y = 0;
	goal_point.x = 1.8;
	goal_point.y = 0;
	Action out_elevator(start_point, goal_point, false);

  out_elevator.move();

  res.status = true;
  return true;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "panel_action_node");
	ros::NodeHandle n;
  
  ros::ServiceServer service = n.advertiseService("elevator_action", elevator_action);
  push_bottun_client_ = n.serviceClient<elevator_navigation_srv::PushBottun>("push_bottun");
  ros::spin();

	return (0);
}
