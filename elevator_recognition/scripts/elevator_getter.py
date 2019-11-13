#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
import cv2

IMAGE_WIDTH = 3260
IMAGE_HEIGHT = 2450

'''
  --- x
 |
 y
  
'''

class elevator_getter():
	def __init__(self):
		self.detection_subscriber = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.bounding_boxes_callback)
	def bounding_boxes_callback(self, detection_data):
		xmin = detection_data.bounding_boxes[0].xmin
		xmax = detection_data.bounding_boxes[0].xmax
		ymin = detection_data.bounding_boxes[0].ymin
		ymax = detection_data.bounding_boxes[0].ymax
		print("x,y", (xmax - xmin)/2, (ymax - ymin)/2)

def main():
	rospy.init_node('elevator_getter')	

	ele = elevator_getter()

	rospy.spin()

if __name__ == '__main__':
	main()
