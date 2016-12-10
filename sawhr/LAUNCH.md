# Launching Simple Autonomous Wheeled Robot (SAWhR) Software Stack

The SAWhR software stack can be launched in a number of different ways, 
depending on your situtation.
Launching is a bit difficult due to the need to work around some race 
conditions in some of the nodes.
In theory you could start all the ROS nodes used using a single launch file.   
In practice, this does not work, at least under Indigo.   
So instead it is recommended to launch the SW stack in phases, 
each phase adding another software layer with a bit more functionality.   
This layering
is also useful if you want to experiment with alternative "higher" layers 
(like an alternative mapping stack)
while leaving the "lower" layers (like motor control) alone, 
or want to test the robot bringup incrementally.

Here are the various ways to start the system.

## Script-Driven Under X11
If you are running using X11, 
either locally (eg with an HDMI cable tether) 
or over the network with "ssh -X", 
you can run the start.sh script.   
This launches all the "phases" with the output redirected to an xterm, 
one xterm per phase,
and with a delay between each phase to avoid race conditions.
The xterms allow you to monitor the output of each phase
in real time, but be aware that if the xterms are terminated those
nodes will close.   
This approach is useful for testing but perhaps not for 
a final install.

## Script-Driven Without X11
If you want to launch the entire software stack without starting xterms, 
you can use the "nox_start.sh" script.
The same approach will be taken with delays between phases as above, 
but output will be directed to the ROS logs.
This is useful if you are logging in remotely with just a shell script 
but no remote X11 server,
or if you want to avoid a dependency on keeping the xterms running.

## Launching Phases Manually
Sometimes you want to test phases separately if you are doing debugging.
You can just launch the phases yourself manually using 
  roslaunch whr init_X.launch
With X one of 0, 1, 2, or 3.  
If you want, 
launch these in separate terminals or ssh sessions to see the output.

## Launch Everything at Once
This launches all the phases at the same time without delays in between.
For the reasons noted above, 
this might not work (it is known not to under Indigo)
but (if certain issues in ROS or the nodes are resolved over time)
may work on your installation.
If you want to, you can try simultaneous launch using:
  roslaunch whr init.launch
Note this does not use a script but just ROS tools.
