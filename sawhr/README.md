# Whr Master Package

This package serves two purposes:
1. To act as a metapackage in that is has 
   dependencies on all other packages needed for the Whr.
   Therefore, you only have to install this package to get everything,
   in theory.
   In practice, some other manual steps are required to set up your
   robot, for example to configure your servos, and to install software
   (such as drivers) not available in the ROS package manager or apt repos.
   See [INSTALL.sh] for details.
2. Includes launch files for startup.
   These are organized in "phases" to resolve ordering dependencies.   
   Launch init_1, then init_2, etc.  It is useful to launch each of these
   in a separate window.  Wait for each to stabilize before starting the 
   next.  You can also use the start.sh script.   
   See [LAUNCH.md] for details.




