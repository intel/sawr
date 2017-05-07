#!/bin/sh
# Launch RealSense multiple camera scan
BASE=`rospack find sawr_master`/scripts
$BASE/xm.sh "roslaunch sawr_scan multi.launch" &

