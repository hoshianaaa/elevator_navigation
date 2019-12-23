#!/usr/bin/env python 

import time
import rospy
import tf
import actionlib
from std_msgs.msg import Float64
from elevator_navigation_msgs.msg import NeckMotionAction, NeckMotionGoal, NeckMotionResult

UP = 0.28
DOWN = 0.65
angle = UP

pub = rospy.Publisher('/neck_controller/command', Float64, queue_size=1)

def handle_neck_motion(goal):
  print("reqest:", goal.number)
  global angle
  global UP
  global DOWN
  global pub

  if goal.number is 0:
    angle = UP

  if goal.number is 1:
    angle = DOWN

  msg = Float64()
  msg.data = angle  
  pub.publish(msg)

  time.sleep(3)

  result = NeckMotionResult()
  server.set_succeeded(result)



rospy.init_node("neck_motion_server")
server = actionlib.SimpleActionServer('neck_motion', NeckMotionAction, handle_neck_motion, False)
server.start()
rospy.spin()



