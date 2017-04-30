#!/bin/sh
echo "Bringing up the SAWR software stack... using saved map"
echo "Start roscore"
BASE=`rospack find sawr_master`/scripts
$BASE/xm.sh roscore &
sleep 15
# make rqt_graph more useful
rosparam set enable_statistics true
# Motor control and odometry
$BASE/xinit.sh 0
sleep 15
# Intel RealSense camera and laser scan emulation
$BASE/xinit.sh 1
# Use the following instead if you have a ZR300
#./scripts/xinit_alt.sh 1
sleep 15
# Mapping and localization
$BASE/xinit_alt.sh 2
sleep 15
# Navigation planning and execution
$BASE/xinit.sh 3
