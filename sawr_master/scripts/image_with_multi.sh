#!/bin/sh
# Test ROS/RealSense integration (with multiple cameras)
BASE=`rospack find sawr_master`/scripts
$BASE/xm.sh "roslaunch sawr_scan multi.launch" &
sleep 10
$BASE/xm.sh "rosrun image_view image_view image:=/cameraL/color/image_raw" & 
$BASE/xm.sh "rosrun image_view image_view image:=/cameraL/depth/image_raw" & 
$BASE/xm.sh "rosrun image_view image_view image:=/camera/color/image_raw" & 
$BASE/xm.sh "rosrun image_view image_view image:=/camera/depth/image_raw" & 
$BASE/xm.sh "rosrun image_view image_view image:=/cameraR/color/image_raw" & 
$BASE/xm.sh "rosrun image_view image_view image:=/cameraR/depth/image_raw" & 
# Following does not work, looks like fisheye camera data is using an OpenCV image format... may
# have to use CV bridge to get this to work
# $BASE/xm.sh "rosrun image_view image_view encoding:=mono16 image:=/camera/fisheye/image_raw" & 

