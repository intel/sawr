#!/bin/sh
# Convert xacro file for fetchbot and whrobot into urdf
rosrun xacro xacro.py xacro/whr.urdf.xacro > urdf/whr.urdf
