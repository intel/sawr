#!/bin/sh
# Test ROS/RealSense integration
xterm -e "roslaunch sawr_scan scan.launch camera_type:=ZR300" &
sleep 10
xterm -e "rosrun image_view image_view image:=/camera/color/image_raw" & 
xterm -e "rosrun image_view image_view image:=/camera/depth/image_raw" & 

