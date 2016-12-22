#!/bin/sh
# Test ROS/RealSense integration
xterm -e "roslaunch realsense_camera r200_nodelet_default.launch" &
sleep 10
xterm -e "rosrun image_view image_view image:=/camera/color/image_raw" & 
xterm -e "rosrun image_view image_view image:=/camera/depth/image_raw" & 

