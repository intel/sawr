#!/bin/sh
# View image from camera (assuming topic already being published;
# see image_with_scan.sh for a script that also launches camera node(s)
# for generating image data)
xterm -e "rosrun image_view image_view image:=/camera/color/image_raw" &

