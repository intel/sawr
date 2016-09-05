#/bin/sh
# Bring up the navigation stack in phases
# Start roscore
xterm -e roscore &
sleep 15
# Motor control and odometry
./scripts/xinit.sh 0
# No need to wait to start teleop
#./scripts/teleop.sh
sleep 15
# Realsense camera and laser scan emulation
./scripts/xinit.sh 1
sleep 15
# Mapping and localization
./scripts/xinit_alt.sh 2
sleep 15
# Navigation planning and execution
./scripts/xinit.sh 3
