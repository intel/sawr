#!/bin/sh
rosrun pcl_ros convert_pointcloud_to_image input:=/camera/depth/points output:=/my_pcl_image &
rosrun image_view image_view image:=/my_pcl_image 

