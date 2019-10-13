#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int8

count = 0
want_go = 0
now_floor = 0
data = 0

ele_state = 0 # 0:stop 1:up 2:down

up_th = 10.3
down_th = 9.3


def IMUcallback(message):
	global count
	global want_go
	global now_floor
	global ele_state
	global data

	if want_go:

		if (count % 10) is 0:	
			data = message.linear_acceleration.z
			print(ele_state,now_floor,data)

			diff_floor = want_go - now_floor
			if (diff_floor > 0):
				if ((ele_state is 0) and (data > up_th)):
					ele_state = 1

				if ((ele_state is 1) and (data < down_th)):
					ele_state = 0
					now_floor = want_go
					print("change floor:", now_floor)
					ch_floor_pub.publish(now_floor)
					want_go = 0
				
			if (diff_floor < 0):
				if ((ele_state is 0) and (data < down_th)):
					ele_state = 2

				if ((ele_state is 2) and (data > up_th)):
					ele_state = 0
					now_floor = want_go
					print("change floor:", now_floor)
					ch_floor_pub.publish(now_floor)
					want_go = 0
	
		count += 1

def wantGo_callback(message):
	global want_go
	want_go = message.data
	print("want to go:", want_go)

rospy.init_node('change_floor')
now_floor = 3
IMUsub = rospy.Subscriber('imu/data_raw', Imu, IMUcallback)
wantGo_sub = rospy.Subscriber('~want_go', Int8, wantGo_callback)
ch_floor_pub = rospy.Publisher('~floor_number', Int8, queue_size=1)
r = rospy.Rate(10)
while not rospy.is_shutdown():
	r.sleep()

