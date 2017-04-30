#/bin/sh
echo "Bringing up the SAWR software stack..."
echo "Start roscore"
BASE=`rospack find sawr_master`/scripts
$BASE/xm.sh roscore &
sleep 15
# make rqt_graph more useful
rosparam set enable_statistics true
echo "Phase 0: Motor control and odometry"
$BASE/xinit.sh 0
sleep 25
echo "Phase 1: Intel RealSense camera and laser scan emulation"
$BASE/xinit.sh 1
# Use the following instead if you have a ZR300
#./scripts/xinit_alt.sh 1
sleep 25
echo "Phase 2: Simultaneous localization and mapping (SLAM)"
$BASE/xinit.sh 2
sleep 25
echo "Phase 3: Navigation planning and execution"
$BASE/xinit.sh 3
