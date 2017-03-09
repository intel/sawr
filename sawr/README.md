# Simple Autonomous Wheeled Robot (SAWR) Master Package

This package serves two purposes:
  1. To act as a metapackage in that is has 
     dependencies on all other packages needed for the SAWR project.
     In theory, you only have to install this package to get all dependencies.
     In practice, some other manual steps are still required to set up your
     robot, for example to configure your servos, and to install software
     (such as drivers) not available in the ROS package manager or apt repos.
     See [INSTALL.md](INSTALL.md) for details.
  2. Includes launch files for startup.
     These are organized in "phases" to resolve ordering dependencies.   
     Launch ``init_1.luanch``, then ``init_2.launch``, etc.
     It is useful to launch each of these
     in a separate window.  
     Wait for each to stabilize before starting the 
     next.  You can also use the ``start.sh`` script.   
     See [LAUNCH.md](LAUNCH.md) for details.




