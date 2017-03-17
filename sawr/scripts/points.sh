#!/bin/sh
# View a pointcloud of the depth and color data produced by the camera
# Note: you need to launch the camera nodes first
rosrun pcl_ros convert_pointcloud_to_image input:=/camera/depth/points output:=/my_pcl_image &
rosrun image_view image_view image:=/my_pcl_image 

