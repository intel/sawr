#!/bin/sh
# Start a keyboard teleop suitable for the SAWR
echo "Starting keyboard teleop"
xterm -e rosrun teleop_twist_keyboard teleop_twist_keyboard.py &

