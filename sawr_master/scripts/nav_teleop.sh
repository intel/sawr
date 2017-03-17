#!/bin/sh
# Start a teleop keyboard that uses the "nav_cmd_vel" channel
rosrun teleop_twist_keyboard teleop_twist_keyboard.py __name:=nav_teleop_twist_keyboard cmd_vel:=nav_cmd_vel

