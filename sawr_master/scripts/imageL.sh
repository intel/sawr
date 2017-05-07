#!/bin/sh
# View image from left camera (assuming topic already being published;
# see multi.sh for a script that also launches camera nodes)
BASE=`rospack find sawr_master`/scripts
$BASE/xm.sh "rosrun image_view image_view image:=/cameraL/color/image_raw" &

