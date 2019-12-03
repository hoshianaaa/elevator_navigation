#!/usr/bin/env python 

import rospy
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Twist


def main():
  rospy.init_node("pub_cmd_vel")
  listener = tf.TransformListener()
  vel_pub = rospy.Publisher('/icart_mini/cmd_vel', Twist, queue_size=1)
  vel_msg = Twist()
  r = rospy.Rate(10)
  '''
  for i in range(10):
    vel_msg.linear.x = 0.003
    vel_msg.angular.z = 0.0
    vel_pub.publish(vel_msg)
    r.sleep()
  '''
  for i in range(39):
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.01
    vel_pub.publish(vel_msg)
    r.sleep()
  '''
  for i in range(10):
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = -0.05
    vel_pub.publish(vel_msg)
    r.sleep()
 '''


if __name__ == '__main__':
  try: 
    print("main")
    main()
  except rospy.ROSInterruptException:
    pass           


