#include <ros/ros.h>
#include <elevator_waypoints_nav/panel_action.h>
#include <elevator_waypoints_nav/action.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <elevator_navigation_srv/ElevatorAction.h>
#include <elevator_navigation_srv/PushBottun.h>
#include <elevator_navigation_srv/NeckMotion.h>
#include <go_target/GoTarget.h>
#include <std_msgs/Int8.h>

int bottun_x_error = 100;
double panel_distance = 100;
int bounding_box_lock = 3;

ros::ServiceClient push_bottun_client_, neck_motion_client_, go_target_client_;
ros::Publisher change_map_pub_;

void neck_motion(int number){
  elevator_navigation_srv::NeckMotion srv;
  srv.request.number = number;
  neck_motion_client_.call(srv);
}

bool go_target_motion(double x, double y, double stop_distance){
  go_target::GoTarget srv;
  srv.request.target_point.x = x;
  srv.request.target_point.y = y;
  srv.request.stop_distance = stop_distance;
  go_target_client_.call(srv);
  return true;
}

bool elevator_action(elevator_navigation_srv::ElevatorAction::Request &req, elevator_navigation_srv::ElevatorAction::Response &res)
{

  double bounding_box_x;
  double go_distance;
  double rotate_angle;
  //in elevator
  double bounding_box_x_in_18 = 330;
  double bounding_box_x_in_3 = 330;
  double bounding_box_x_in_1 = 320;
  double go_distance_in_18 = 0.07;
  double go_distance_in_3 = 0.07;
  double go_distance_in_1 = 0.1;
  double rotate_angle_in_18 = M_PI/7;
  double rotate_angle_in_3 = M_PI/7;
  double rotate_angle_in_1 = M_PI/6;

  double up_arm_error_th = 0.01;

 if (req.start_floor == 18){
    bounding_box_x = bounding_box_x_in_18;
    go_distance = go_distance_in_18;
    rotate_angle = rotate_angle_in_18;
  }
  if (req.start_floor == 3){
    bounding_box_x = bounding_box_x_in_3;
    go_distance = go_distance_in_3;
    rotate_angle = rotate_angle_in_3;
  }
  if (req.start_floor == 1){
    bounding_box_x = bounding_box_x_in_1;
    go_distance = go_distance_in_1;
    rotate_angle = rotate_angle_in_1;
  }

  neck_motion(0);

	PanelAction pa_up("up");
  pa_up.home_arm();
///*
  double elevator_in_panel_pos[] = {1.85, -0.8};
  go_target_motion(elevator_in_panel_pos[0], elevator_in_panel_pos[1], 0.6);
 
  int bounding_box_stop_number = 1;

  //3kai up 1.275 
  //3kai down 1.22
  //1kai up 1.30
  //1kai down 1.25
  //18kai up 1.23
  //18kai down 1.18
  double up_3kai = 1.24;
  double down_3kai = 1.2;
  double up_1kai = 1.26;
  double down_1kai = 1.22;
  double up_18kai = 1.23;
  double down_18kai = 1.17;

  int start_arm_number = 5;
  int up_3kai_arm_number = 9;
  int down_3kai_arm_number = 15;
  int up_1kai_arm_number = 9;
  int down_1kai_arm_number = 15;
  int up_18kai_arm_number = 9;
  int down_18kai_arm_number = 15;

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
      start_arm_number = up_3kai_arm_number;
    }
    else if(floor == 18){
      height = up_18kai;
      start_arm_number = up_18kai_arm_number;
    }
    else if(floor == 1){
      height = up_1kai;
      start_arm_number = up_1kai_arm_number;
    }
    else{
      ROS_ERROR("no floor");
    }
  }
  else{
    if(floor == 3){
      height = down_3kai;
      start_arm_number = down_3kai_arm_number;
    }
    else if(floor == 18){
      height = down_18kai;
      start_arm_number = down_18kai_arm_number;
    }
    else if(floor == 1){
      height = down_1kai;
      start_arm_number = down_1kai_arm_number;
    }
    else{
      ROS_ERROR("no floor");
    }
  }

  std::cout << "heigth:" << height << std::endl;

  double stop_distance = 0.50;


  int id = 0;

  int track_b_box_id;
  if(up)track_b_box_id = 1;
  if(!up)track_b_box_id = 2;

  pa_up.track_b_box_start(track_b_box_id);
  while(1){

    if(pa_up.rotate_for_bounding_box(bounding_box_x, 0, 5))break;
    if(pa_up.go_panel(stop_distance))break;
    if(pa_up.up_arm(height, up_arm_error_th, start_arm_number))break;
    pa_up.check_door_start();
    if(pa_up.straight(go_distance))break;
    if(pa_up.straight(-go_distance))break;
    pa_up.home_arm();

    go_distance += 0.01;
  }

  geometry_msgs::Point start_point, goal_point;

  pa_up.track_b_box_clear();
  pa_up.home_arm();
  pa_up.back(0.7, 0.5);
 	start_point.x = 1.8;
	start_point.y = 0;
	goal_point.x = 0;
	goal_point.y = -0.3;

  Action in_elevator(start_point, goal_point, true);
  in_elevator.move();
  //*/
  //out elevator
  double rotate_out = -M_PI/4;
  pa_up.home_arm();
	PanelAction pa_e18("e18");
	PanelAction pa_e1("e1");

  double elevator_out_panel_pos[] = {1.0, -0.8};
  go_target_motion(elevator_out_panel_pos[0], elevator_out_panel_pos[1], 0.6);

  int bounding_box_x2 = 315;
  int bounding_box_stop_number2 = 3;
  double height_e18 = 1.26;
  double height_e1 = 0.98;
  double go_distance_1kai = 0.1;
  double stop_distance_1kai = 0.45;
  double stop_distance2 = 0.45;
  double go_distance2 = 0.09;


  if (req.target_floor == 1)
  {
    neck_motion(1);
    pa_e1.home_arm_down();
  }

  pa_e1.track_b_box_start(1);
  //pa_e1.check_door_start();

  pa_e18.track_b_box_start(1);
  //pa_e18.check_door_start();

  while(1){
    if (req.target_floor == 1){

      pa_e1.home_arm_down();
      if(pa_e1.rotate_for_bounding_box(bounding_box_x2))break;
      if(pa_e1.go_panel(stop_distance_1kai))break;
      pa_e1.down_arm(height_e1, up_arm_error_th);
      //pa_e1.check_door_start();
      if(pa_e1.straight(go_distance_1kai))break;
      if(pa_e1.straight(-go_distance_1kai))break;
      pa_e1.home_arm_down();
      sleep(3);
      go_distance_1kai += 0.01;
    }
    else{
      if(pa_e18.rotate_for_bounding_box(bounding_box_x2))break;
      if(pa_e18.go_panel(stop_distance2))break;
      pa_e18.up_arm(height_e18, up_arm_error_th, 1);
      //pa_e18.check_door_start();
      if(pa_e18.straight(go_distance2))break;
      std::cout << "back!" << std::endl;
      if(pa_e18.straight(-go_distance2))break;
      pa_e18.home_arm();
      sleep(3);
      go_distance_1kai += 0.01;
    }
  }

  pa_e18.back(0.5, 0.5);
  pa_e18.rotate_home();
  
  double OPEN_DOOR_SENSOR_SUM_TH = 1000;
  if(pa_e18.get_scan_sum() < OPEN_DOOR_SENSOR_SUM_TH)pa_up.home_arm();

  geometry_msgs::Point start_point2, goal_point2;

 	start_point2.x = 0;
	start_point2.y = 0;
	goal_point2.x = 4;
	goal_point2.y = 0;
	Action out_elevator(start_point2, goal_point2, false);

  std::cout << "debug out elevator start" << std::endl;
  out_elevator.move();

  std_msgs::Int8 msg;
  msg.data = req.target_floor;
  change_map_pub_.publish(msg);

  std::cout << "debug out elevator finish" << std::endl;
  

  pa_up.home_arm();
  res.status = true;
  return true;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "panel_action_node");
	ros::NodeHandle n;
  
  change_map_pub_ = n.advertise<std_msgs::Int8>("change_floor/floor_number", 1);
  ros::ServiceServer service = n.advertiseService("elevator_action", elevator_action);
  push_bottun_client_ = n.serviceClient<elevator_navigation_srv::PushBottun>("push_bottun");
  neck_motion_client_ = n.serviceClient<elevator_navigation_srv::NeckMotion>("neck_motion");
  go_target_client_ = n.serviceClient<go_target::GoTarget>("go_target");
  ros::spin();

	return (0);
}
