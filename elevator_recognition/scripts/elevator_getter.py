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
  
'''

class elevator_getter():
	def __init__(self):
		self.bridge = CvBridge()
		self.bounding_box_subscriber = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.bounding_boxes_callback)
		#self.depth_image_subscriber = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_image_callback)
		self.darknet_pub = rospy.Publisher("darknet_data", Int32, queue_size=1)
		self.image_panel_depth = 0
		self.bounding_box_x = 0
		self.bounding_box_y = 0

	def bounding_boxes_callback(self, msg):
		xmin = msg.bounding_boxes[0].xmin
		xmax = msg.bounding_boxes[0].xmax
		ymin = msg.bounding_boxes[0].ymin
		ymax = msg.bounding_boxes[0].ymax
		self.bounding_boxes_x = (xmax - xmin)/2 + xmin
		self.bounding_boxes_y = (ymax - ymin)/2 + ymin
		#print(self.bounding_boxes_x, self.bounding_boxes_y)
		self.darknet_pub.publish(int(self.bounding_boxes_x - IMAGE_WIDTH/2))
		print("publish darknet data")
	'''
	def depth_image_callback(self, msg):
		try:
			depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')	
		except CvBridgeError, e:
			rospy.logerr(e)
			
		
		print(depth_image[self.bounding_boxes_y, self.bounding_boxes_x])
	'''
		
		
		

def main():
	rospy.init_node('elevator_getter')	

	ele = elevator_getter()

	rospy.spin()

if __name__ == '__main__':
	main()
