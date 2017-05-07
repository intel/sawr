#!/bin/sh
# View image from right camera (assuming topic already being published;
# see multi.sh for a script that also launches camera nodes)
BASE=`rospack find sawr_master`/scripts
$BASE/xm.sh "rosrun image_view image_view image:=/cameraR/color/image_raw" &

