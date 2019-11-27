#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from geometry_msgs.msg import Point
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

'''
  --- x
 |
 y

publish: bounding box x,y coordinate  
'''
 
class bounding_box_listener():
  def __init__(self):
    self.bounding_box_subscriber = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.bounding_boxes_callback)
    self.bounding_box_pos_pub = rospy.Publisher("bounding_box_pos", Point, queue_size=1)

  def bounding_boxes_callback(self, msg):
    print("call back")

    xmin = msg.bounding_boxes[0].xmin
    xmax = msg.bounding_boxes[0].xmax
    ymin = msg.bounding_boxes[0].ymin
    ymax = msg.bounding_boxes[0].ymax

    bounding_box_x = float(int((xmax - xmin)/2 + xmin))
    bounding_box_y = float(int((ymax - ymin)/2 + ymin))
  
    self.bounding_box_pos_pub.publish([bounding_box_x, bounding_box_y, 0])

def main():
  rospy.init_node('bounding_box_listener')	
  bl = bounding_box_listener()
  rospy.spin()

if __name__ == '__main__':
  main()
