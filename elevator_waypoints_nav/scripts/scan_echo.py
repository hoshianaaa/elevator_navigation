#!/usr/bin/env python 

import rospy
import tf
import sensor_msgs.msg
from sensor_msgs.msg import LaserScan


def callback(data):
  print("scan middle val:")
  print(data.ranges[len(data.ranges)/2])

def main():
  rospy.init_node("pub_cmd_vel")
  rospy.Subscriber("scan", LaserScan, callback)
  rospy.spin()

  

if __name__ == '__main__':
  print("main")
  main()


