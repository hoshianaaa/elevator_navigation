#!/usr/bin/env python 

import rospy
import tf
from std_msgs.msg import Float64
from elevator_navigation_srv.srv import NeckMotion

UP = 0.28
DOWN = 0.65
angle = UP

pub = rospy.Publisher('/neck_controller/command', Float64, queue_size=1)

def handle_neck_motion(req):
  print("req", req.number)
  global angle
  global UP
  global DOWN
  global pub

  if req.number is 0:
    angle = UP

  if req.number is 1:
    angle = DOWN

  msg = Float64()
  msg.data = angle  
  pub.publish(msg)

  return 1


def main():
  rospy.init_node("neck_server")
  s = rospy.Service('neck_motion', NeckMotion, handle_neck_motion)
  rospy.spin()
  
if __name__ == '__main__':
  try: 
    print("main")
    main()
  except rospy.ROSInterruptException:
    pass           


