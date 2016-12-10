#!/bin/sh
# Convert xacro file for xacro into urdf
rosrun xacro xacro.py xacro/sawhr.urdf.xacro > urdf/sawhr.urdf
