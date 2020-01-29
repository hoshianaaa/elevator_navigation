#!/bin/bash

source ${HOME}/catkin_ws/devel/setup.bash
gnome-terminal -- bash -c "roslaunch elevator_recognition darknet_ros_up.launch"
sleep 5
gnome-terminal -- bash -c "roslaunch elevator_recognition darknet_ros_e18.launch"
sleep 5
gnome-terminal -- bash -c "roslaunch elevator_recognition darknet_ros_e1.launch"
sleep 8
gnome-terminal -- bash -c "roslaunch freenect_launch freenect.launch"
