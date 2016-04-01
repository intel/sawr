#!/bin/sh
# Convert xacro file for fetchbot into urdf
rosrun xacro xacro.py xacro/fetchbot.urdf.xacro > urdf/fetchbot.urdf
