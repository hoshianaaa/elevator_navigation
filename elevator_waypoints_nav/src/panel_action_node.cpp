#include <ros/ros.h>
#include <elevator_waypoints_nav/panel_action.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

int bottun_x_error = 100;
double panel_distance = 100;
int bounding_box_lock = 3;

void boundingBoxCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  std::cout << "bounding box call back" << std::endl;
  bottun_x_error = msg->x - 340;
  //std::cout << "error:" << msg->x - 340 << std::endl;
  //std::cout << "bottun_x_error:" << bottun_x_error << std::endl;
  if(bounding_box_lock>0)bounding_box_lock--;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "panel_action_node");
	ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("bounding_box_pos", 1, boundingBoxCallback);
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("icart_mini/cmd_vel", 1);
	PanelAction pa;
	pa.rotate(M_PI/8);
	pa.go_panel(0.45);
  ros::Rate r(10);
  geometry_msgs::Twist vel;
  int orientation = 0;
  int p_bounding_box_lock = 0;
  ros::Time unchange_bounding_box_timer;
  int unchange_timer;
  while(1){
    if(p_bounding_box_lock != bounding_box_lock){
      unchange_bounding_box_timer = ros::Time::now();
    }
    unchange_timer =  ros::Time::now().toSec() - unchange_bounding_box_timer.toSec();
    std::cout << "unchange time:" << unchange_timer << std::endl;
    int b = 0;
    
    if ((bounding_box_lock == 0) || unchange_timer > 10){
      unchange_bounding_box_timer = ros::Time::now();
      std::cout << "publish vel" << std::endl;
      if(bottun_x_error < 5 && bottun_x_error > -5){b = 1;break;}
      vel.angular.z = -bottun_x_error/100.0; 

      if (vel.angular.z > 0.2){
        vel.angular.z = 0.2;
        orientation = 1;
      }
      else if (vel.angular.z < -0.2){
        vel.angular.z = -0.2;
        orientation = -1;
      }
      std::cout << vel.angular.z << std::endl;
      bounding_box_lock = 4;
    }

    if (vel.angular.z>0)orientation++;
    else if(vel.angular.z<0)orientation--;

    int limit = 3;
    if(orientation>limit){vel.angular.z=0;orientation=0;}
    else if(orientation<-limit){vel.angular.z=0;orientation=0;}
    
    if(b)break;
    p_bounding_box_lock = bounding_box_lock;

    std::cout << "vel:" << vel.angular.z << " ori:" << orientation << " error:" << bottun_x_error << std::endl;
    vel_pub.publish(vel);
    ros::spinOnce();
    r.sleep();
  }
	pa.go_panel(0.45);
  //pa.straight(0.01);
	return (0);
}
