#!/bin/sh
# Launch RealSense scan
BASE=`rospack find sawr_master`/scripts
$BASE/xm.sh "roslaunch sawr_scan scan.launch" &

