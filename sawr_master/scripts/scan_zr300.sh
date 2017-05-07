#!/bin/sh
# Launch RealSense scan for ZR300 camera
BASE=`rospack find sawr_master`/scripts
$BASE/xm.sh "roslaunch sawr_scan scan_zr300.launch" &

