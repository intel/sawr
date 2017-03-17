#!/bin/sh
# Test ROS/RealSense integration (with default camera, an R200)
xterm -e "roslaunch sawr_scan scan.launch" &
sleep 10
xterm -e "rosrun image_view image_view image:=/camera/color/image_raw" & 
xterm -e "rosrun image_view image_view image:=/camera/depth/image_raw" & 

