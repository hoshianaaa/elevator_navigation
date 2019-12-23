#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

'''
  --- x
 |
 y

publish: bounding box x,y coordinate  
'''
 
class bounding_box_listener():
  def __init__(self, name):
    sub_topic_name = "/darknet_ros_" + name + "/bounding_boxes"
    pub_topic_name = name + "bounding_box_pos"
    self.bounding_box_subscriber = rospy.Subscriber(sub_topic_name, BoundingBoxes, self.bounding_boxes_callback)
    self.bounding_box_pos_pub = rospy.Publisher(pub_topic_name, PoseArray, queue_size=1)

  def bounding_boxes_callback(self, msg):
    #print("call back")
    
    poses = PoseArray()
    pose = Pose()
    for i in range(len(msg.bounding_boxes)):

      xmin = msg.bounding_boxes[i].xmin
      xmax = msg.bounding_boxes[i].xmax
      ymin = msg.bounding_boxes[i].ymin
      ymax = msg.bounding_boxes[i].ymax

      bounding_box_x = float(int((xmax - xmin)/2 + xmin))
      bounding_box_y = float(int((ymax - ymin)/2 + ymin))

      pose.position.x = bounding_box_x
      pose.position.y = bounding_box_y
      pose.position.z = msg.bounding_boxes[i].id

      poses.poses.append(pose)
   
    self.bounding_box_pos_pub.publish(poses)

def main():
  rospy.init_node('bounding_box_listener')	
  bl_up = bounding_box_listener("up")
  bl_e18 = bounding_box_listener("e18")
  bl_e1 = bounding_box_listener("e1")
  rospy.spin()

if __name__ == '__main__':
  main()
