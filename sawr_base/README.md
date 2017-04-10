Simple Autonomous Wheeled Robot (SAWR)
======================================
Base Package
------------

This package provide base system services for the SAWR including:
  1. Control of the Robotis motors via the [Dynamixel SDK][DSDK].
  2. Odometry, by integration of the motor velocities. The code used for this 
     is an elaboration of the [ROS Odometry Setup Tutorial][Odom].
  3. Battery level sensing (using the voltage sensing provided by the motors).

Movement commands should be published as twists on the `cmd_vel` topic and 
odometry will be published on the `odom` topic. An odometry transform is also 
published.

[DSDK]: https://github.com/ROBOTIS-GIT/DynamixelSDK
[Odom]: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
