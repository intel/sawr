#!/bin/sh
echo "Starting keyboard teleop"
xterm -e rosrun teleop_twist_keyboard teleop_twist_keyboard.py &

