#/bin/sh
echo "Bringing up the SAWR software stack..."
echo "Start roscore"
xterm -e roscore &
sleep 15
# make rqt_graph more useful
rosparam set enable_statistics true
echo "Phase 0: Motor control and odometry"
./scripts/xinit.sh 0
sleep 25
echo "Phase 1: Intel RealSense camera and laser scan emulation"
./scripts/xinit.sh 1
sleep 25
echo "Phase 2: Simultaneous localization and mapping (SLAM)"
./scripts/xinit.sh 2
sleep 25
echo "Phase 3: Navigation planning and execution"
./scripts/xinit.sh 3
