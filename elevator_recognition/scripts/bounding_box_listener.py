#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from std_msgs.msg import Int32
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
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
    self.bounding_box_x_pub = rospy.Publisher("darknet_data_x", Int32, queue_size=1)
    self.bounding_box_y_pub = rospy.Publisher("darknet_data_y", Int32, queue_size=1)

  def bounding_boxes_callback(self, msg):

    xmin = msg.bounding_boxes[0].xmin
    xmax = msg.bounding_boxes[0].xmax
    ymin = msg.bounding_boxes[0].ymin
    ymax = msg.bounding_boxes[0].ymax

    bounding_box_x = int((xmax - xmin)/2 + xmin)
    bounding_box_y = int((ymax - ymin)/2 + ymin)

    self.bounding_box_x_pub.publish(bounding_box_x)
    self.bounding_box_y_pub.publish(bounding_box_y)

def main():
  rospy.init_node('bounding_box_listener')	
  bl = bounding_box_listener()
  rospy.spin()

if __name__ == '__main__':
  main()
