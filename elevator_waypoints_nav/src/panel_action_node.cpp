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
  
  elevator_navigation_srv::PushBottun srv;
  srv.request.bounding_box_x = 340;
  srv.request.bounding_box_stop_number = 1;
  srv.request.height = 1.275;
  srv.request.stop_distance = 0.45;
  srv.request.go_distance = 0.05;
  push_bottun_client_.call(srv);

	geometry_msgs::Point start_point, goal_point;
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
	Action near_elevator(start_point, goal_point, false);

  near_elevator.rotate(0.3, 0.3);
  near_elevator.move();

 	start_point.x = 1.8;
	start_point.y = 0;
	goal_point.x = 0;
	goal_point.y = -0.3;
	Action in_elevator(start_point, goal_point, true);

  in_elevator.move();

  //out elevator
	PanelAction pa2;
	pa2.rotate(-M_PI/6);
	pa2.go_panel(0.45);
  pa2.rotate_for_bounding_box(340);
  pa2.go_panel(0.45);
  
  srv.request.bounding_box_x = 330;
  srv.request.bounding_box_stop_number = 3;
  srv.request.height = 1.3;
  srv.request.stop_distance = 0.45;
  srv.request.go_distance = 0.09;
  push_bottun_client_.call(srv);

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
