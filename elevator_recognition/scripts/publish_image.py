#!/usr/bin/env python
import os
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

def operator():
	rospy.init_node('operator', anonymous=True)
	pub = rospy.Publisher('/camera/rgb/image_color', Image, queue_size=10)
	# read image
	#filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'e18.jpg')
	filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'up.jpg')
	im = cv2.imread(filepath, cv2.IMREAD_COLOR)
	# make bridge
	bridge = CvBridge()
	msg = bridge.cv2_to_imgmsg(im, encoding="bgr8")
	rate = rospy.Rate(1) # 1hz
	while not rospy.is_shutdown():
		pub.publish(msg)
		rate.sleep()
		print("pub")

if __name__ == '__main__':
	try:
		operator()
	except rospy.ROSInterruptException:
		pass
