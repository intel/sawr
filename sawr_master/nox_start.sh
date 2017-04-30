#/bin/sh
echo "Bringing up the SAWR software stack... No X11"
echo "Start roscore"
BASE=`rospack find sawr_master`/scripts
roscore &
sleep 15
# make rqt_graph more useful
# rosparam set enable_statistics true
echo "Phase 0: Motor control and odometry"
$BASE/init.sh 0 
sleep 25
echo "Phase 1: Intel RealSense camera and laser scan emulation"
$BASE/init.sh 1 
# Use the following instead if you have a ZR300
# ./scripts/init_alt.sh 1 
sleep 25
echo "Phase 2: Simultaneous localization and mapping (SLAM)"
$BASE/init.sh 2 
sleep 25
echo "Phase 3: Navigation planning and execution"
$BASE/init.sh 3 
