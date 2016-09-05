#!/bin/sh
# Convert xacro file for xacro into urdf
rosrun xacro xacro.py xacro/whr.urdf.xacro > urdf/whr.urdf
rosrun xacro xacro.py xacro/trw.urdf.xacro > urdf/trw.urdf
