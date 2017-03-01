#/bin/sh
echo "Bringing up the SAWR software stack... No X"
echo "Start roscore"
roscore &
sleep 15
# make rqt_graph more useful
# rosparam set enable_statistics true
echo "Phase 0: Motor control and odometry"
./scripts/init.sh 0 
sleep 25
echo "Phase 1: Intel RealSense camera and laser scan emulation"
./scripts/init.sh 1 
sleep 25
echo "Phase 2: Simultaneous localization and mapping (SLAM)"
./scripts/init.sh 2 
sleep 25
echo "Phase 3: Navigation planning and execution"
./scripts/init.sh 3 
