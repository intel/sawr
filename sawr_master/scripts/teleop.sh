#!/bin/sh
# Start a keyboard teleop suitable for the SAWR
echo "Starting keyboard teleop"
BASE=`rospack find sawr_master`/scripts
$BASE/xt.sh "rosrun teleop_twist_keyboard teleop_twist_keyboard.py" &
