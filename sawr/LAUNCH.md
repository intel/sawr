# Launching Simple Autonomous Wheeled Robot (SAWR) Software Stack

The SAWR software stack can be launched in a number of different ways,
depending on your situtation.
Launching is a bit difficult due to the need to work around some race
conditions in some of the nodes.
In theory you could start all the ROS nodes used using a single launch file.
In practice, this sometimes does not work.
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
nodes will close (and also, if the nodes crash, the xterms will close).
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

    roslaunch sawr init_X.launch

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

    roslaunch sawr init.launch

Note this does not use a script but just ROS tools.

## Teleop and Visualization

Now that the stack is launched, you can "drive" the robot using a teleoperator
and visualize the output with rviz.   Do the following in two separate windows:

    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    roslaunch sawr_navigation display.launch

## Remote Access

If you set up remote access via ssh as noted in [INSTALL.md](INSTALL.md),
you will be able to launch the above remotely.
If you use the -X option when you log in via ssh,
even the X11 versions with multiple terminal windows will
work---definitely on a remote Linux or OS/X system but even from Windows if you
install an X server such as [Xming](http://www.straightrunning.com/XmingNotes/).
However note that rviz will not work remotely over X11,
so to use it remotely you will have to either set up remote ROS access
(which will be insecure unless you also set up a VPN like OpenVPN)
or set up a VNC server on your robot
(less efficient, but can be tunnelled over ssh).

## Launching Automatically at Boot

The simplest way to launch the system automatically at boot is to enable automatic login
and then configure it as a [Startup Application](https://help.ubuntu.com/16.04/ubuntu-help/startup-applications.html).
Note that you may have to run it inside a terminal window to get this to work.
This is somewhat insecure but you can always have your login time out quickly.
However, note that anyone with physical access to your robot can typically break in without too much trouble
even if you don't set up automatic login.

You can also configure a
[script to run at boot using systemd](https://linuxconfig.org/how-to-automatically-execute-shell-script-at-startup-boot-on-systemd-linux).
This is a bit more robust and will work even without automatic login and even if you disable the windowing system,
but is a little harder to set up.
