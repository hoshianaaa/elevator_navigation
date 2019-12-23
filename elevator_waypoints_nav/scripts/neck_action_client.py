#!/usr/bin/env python 

import rospy
import actionlib
from std_msgs.msg import Float64
from elevator_navigation_msgs.msg import NeckMotionAction, NeckMotionGoal, NeckMotionResult

rospy.init_node("neck_motion_client")

client = actionlib.SimpleActionClient('neck_motion', NeckMotionAction)
client.wait_for_server()
goal = NeckMotionGoal()
goal.number = 0
client.send_goal(goal)
client.wait_for_result()
goal.number = 1
client.send_goal(goal)
client.wait_for_result()
goal.number = 0
client.send_goal(goal)
client.wait_for_result()





