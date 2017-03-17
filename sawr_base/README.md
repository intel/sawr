# Simple Autonomous Wheeled Robot (SAWR) Base Package

This package provide base system services for the SAWR including:
  1. Control of the Robotis motors via the [Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK).
  2. Odometry, by integration of the motor velocities. The code used for this is an elaboration of the [ROS Odometry Setup Tutorial](http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom).
  3. Battery level sensing (using the voltage sensing provided by the motors).

Movement commands should be published as twists on ``cmd_vel`` and odometry will be published on ``odom``.
An odometry transform is also published.
