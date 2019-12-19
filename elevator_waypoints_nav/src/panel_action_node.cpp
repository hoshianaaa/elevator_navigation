#include <ros/ros.h>
#include <elevator_waypoints_nav/panel_action.h>
#include <elevator_waypoints_nav/action.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <elevator_navigation_srv/ElevatorAction.h>
#include <elevator_navigation_srv/PushBottun.h>
#include <elevator_navigation_srv/NeckMotion.h>

int bottun_x_error = 100;
double panel_distance = 100;
int bounding_box_lock = 3;

ros::ServiceClient push_bottun_client_, neck_motion_client_;

void neck_motion(int number){
  elevator_navigation_srv::NeckMotion srv;
  srv.request.number = number;
  neck_motion_client_.call(srv);
}

bool elevator_action(elevator_navigation_srv::ElevatorAction::Request &req, elevator_navigation_srv::ElevatorAction::Response &res)
{

  neck_motion(0);

	PanelAction pa;
  //in elevator
	pa.rotate(M_PI/8);
	pa.go_panel(0.47);
  pa.rotate_for_bounding_box(330);
  pa.go_panel(0.47);
  
  int bounding_box_x = 340;
  int bounding_box_stop_number = 1;

  //3kai up 1.275 
  //3kai down 1.22
  //1kai up 1.30
  //1kai down 1.25
  //18kai up 1.23
  //18kai down 1.18
  double up_3kai = 1.24;
  double down_3kai = 1.2;
  double up_1kai = 1.30;
  double down_1kai = 1.25;
  double up_18kai = 1.23;
  double down_18kai = 1.18;

  int floor;
  bool up;
  double height;

  if (req.start_floor == 1)floor = 1;
  if (req.start_floor == 3)floor = 3;
  if (req.start_floor == 18)floor = 18;

  if (req.target_floor > req.start_floor)up = 1;
  else up = 0;

  if(up){
    if(floor == 3){
      height = up_3kai;
    }
    else if(floor == 18){
      height = up_18kai;
    }
    else if(floor == 1){
      height = up_1kai;
    }
    else{
      ROS_ERROR("no floor");
    }
  }
  else{
    if(floor == 3){
      height = down_3kai;
    }
    else if(floor == 18){
      height = down_18kai;
    }
    else if(floor == 1){
      height = down_1kai;
    }
    else{
      ROS_ERROR("no floor");
    }
  }

  std::cout << "heigth:" << height << std::endl;

  double stop_distance = 0.47;
  double go_distance = 0.07;

  double first_scan_sum, scan_sum;
  first_scan_sum = pa.get_scan_sum();
  std::cout << std::endl << std::endl <<  "first scan sum!!!!!!!:" << first_scan_sum << std::endl;

  int id = 0;
  while(1){

    pa.rotate_for_bounding_box(bounding_box_x);
    std::cout << "go panel" << std::endl;
    pa.go_panel(stop_distance);
    std::cout << "up arm" << std::endl;
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
    if (pa.find_bounding_box(1)){
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
  pa2.rotate_for_bounding_box(340, 2);
  pa2.go_panel(0.45);

  int bounding_box_x2 = 330;
  int bounding_box_stop_number2 = 3;
  double height2 = 1.27;
  double height_1kai = 0.98;
  double go_distance_1kai = 0.08;
  double stop_distance_1kai = 0.45;
  double stop_distance2 = 0.45;
  double go_distance2 = 0.09;


  if (req.target_floor == 1)
  {
    neck_motion(1);
  }

  while(1){
    if (req.target_floor == 1){
      pa2.home_arm_down();
      pa2.rotate_for_bounding_box(bounding_box_x2, 4);
      pa2.go_panel(stop_distance_1kai);
      pa2.down_arm(height_1kai, 0.005);
      pa2.straight(go_distance_1kai);
      pa2.straight(-go_distance_1kai);
      pa2.home_arm_down();
      if (pa2.find_bounding_box(5))break;
    }
    else{
      pa2.rotate_for_bounding_box(bounding_box_x2, 2);
      pa2.go_panel(stop_distance2);
      pa2.up_arm(height2, 0.005, 1);
      pa2.straight(go_distance2);
      pa2.straight(-go_distance2);
      pa2.home_arm();
      if (pa2.find_bounding_box(3))break;
    }
  }

  pa2.back(0.50);
  pa2.rotate_home();

  geometry_msgs::Point start_point2, goal_point2;

 	start_point2.x = 0;
	start_point2.y = 0;
	goal_point2.x = 4;
	goal_point2.y = 0;
	Action out_elevator(start_point2, goal_point2, false);

  out_elevator.move();

  res.status = true;
  return true;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "panel_action_node");
	ros::NodeHandle n;
  
  ros::ServiceServer service = n.advertiseService("elevator_action", elevator_action);
  push_bottun_client_ = n.serviceClient<elevator_navigation_srv::PushBottun>("push_bottun");
  neck_motion_client_ = n.serviceClient<elevator_navigation_srv::NeckMotion>("neck_motion");
  ros::spin();

	return (0);
}
