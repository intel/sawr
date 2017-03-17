#!/bin/sh
# View depth image from camera (assuming topic already being published;
# see image_with_scan.sh for a script that also launches a camera nodes
# for generating depth data)
xterm -e "rosrun image_view image_view image:=/camera/depth/image_raw" & 

