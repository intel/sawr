#!/bin/sh
echo "Bringing up the SAWR software stack... using saved map"
echo "Start roscore"
xterm -e roscore &
sleep 15
# make rqt_graph more useful
rosparam set enable_statistics true
# Motor control and odometry
./scripts/xinit.sh 0
sleep 15
# Intel RealSense camera and laser scan emulation
./scripts/xinit.sh 1
# Use the following instead if you have a ZR300
#./scripts/xinit_alt.sh 1
sleep 15
# Mapping and localization
./scripts/xinit_alt.sh 2
sleep 15
# Navigation planning and execution
./scripts/xinit.sh 3
