#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
count = 0
def callback(message):
	global count
	if (count % 5) is 0:	
		print(message.linear_acceleration.z)
	count += 1

rospy.init_node('imu_listener')
sub = rospy.Subscriber('imu/data_raw', Imu, callback)
rospy.spin()

