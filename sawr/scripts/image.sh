#!/bin/sh
# View image from camera (assuming topic already being published;
# see image_with_r200nodelet.sh for a script that also launches a nodelet
# for generating image data)
xterm -e "rosrun image_view image_view image:=/camera/color/image_raw" &

