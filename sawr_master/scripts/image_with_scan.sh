#!/bin/sh
# Test ROS/RealSense integration (with default camera, an R200)
BASE=`rospack find sawr_master`/scripts
$BASE/xm.sh "roslaunch sawr_scan scan.launch" &
sleep 10
$BASE/xm.sh "rosrun image_view image_view image:=/camera/color/image_raw" & 
$BASE/xm.sh "rosrun image_view image_view image:=/camera/depth/image_raw" & 

