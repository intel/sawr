#!/bin/sh
# Convert xacro file for xacro into urdf
rosrun xacro xacro.py xacro/sawr.urdf.xacro > urdf/sawr.urdf
