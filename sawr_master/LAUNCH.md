Simple Autonomous Wheeled Robot (SAWR)
======================================
Launching the Software Stack
------------------------

The SAWR software stack supports autonomous navigation using simultaneous 
localization and mapping (SLAM). This means that it is capable of creating a 
model of its environment (a map) and then figuring out its current location 
with respect to that map (localization). In addition, autonomous navigation 
is supported. This means the robot can be given a goal destination in the map,
then it can plan a path from its current estimated position to the destination,
and then execute the plan. The plan of course will route around known obstacles
in the map but during execution the robot will also dynamically avoid obstacles
that show up along the way, re-planning as necessary.

![How Robot Navigation Works](http://wiki.ros.org/navigation?action=AttachFile&do=get&target=nav_comic.png)

This is a complex system but is the basis of many other applications in mobile
robotics. To manage complexity, the software system is built up from a number
of phases or "layers". 

* The first layer just supports control of the motors,
  allowing the robot to move forward and turn with specific velocities, and to
  estimate actual progress using odometry. 
* The second layer enables the Intel&reg; RealSense&trade; 3D cameras and
  generates data that simulates a LIDAR.
* The third layer takes this data and implements SLAM, building a map and
  localizing.
* The fourth layer does planning and navigation.

Finally, to control the robot and see where it is, we can use a visualization
tool to observe the map and give the robot navigation goals, and can also
use "teleoperation" to directly drive the robot from place to place.

The layers are built up from standard ROS nodes for SLAM and navigation.
In particular, we use the [ROS Kinetic Kame](http://wiki.ros.org/kinetic)
release, the ROS [slam_gmapping](http://wiki.ros.org/slam_gmapping) package
for mapping, [amcl](http://wiki.ros.org/amcl) package for probablistic
localization, and the ROS [move-base](http://wiki.ros.org/move_base) package
for navigation.  This is similar to the stack used in the 
[ROS navigation](http://wiki.ros.org/navigation) package.

The SAWR software stack can be launched in a number of different ways,
depending on your situation. In theory you could start all the ROS nodes used 
using a single launch file. In practice, this sometimes does not work due to
race conditions in some of the nodes. So instead it is recommended to launch
the SW stack in phases, each phase adding another software layer with a bit
more functionality.

Breaking the launch process into phases is also useful if you want to
experiment with alternative "higher" layers (like an alternative SLAM stack)
while leaving the "lower" layers (like motor control) alone, or just want to
test the robot bringup incrementally.

Here are the various ways to start the system.

Script-Driven Under X11
-----------------------

If you are running using X11, either locally (eg with an HDMI cable tether)
or over the network with `ssh -X`, you can run the `start.sh` script.
This launches all the "phases" with the output redirected to an xterm,
one xterm per phase, and with a delay between each phase to avoid race
conditions.

The xterms allow you to monitor the output of each phase
in real time, but be aware that if the xterms are terminated those
nodes will close (and also, if the nodes crash, the xterms will close).
This approach is useful for testing but perhaps not for a final install.

To take this approach, issue the following commands, either from the main 
X11 display or over an X-tunnelled ssh session:
    
    roscd sawr_master
    ./start.sh

Script-Driven Without X11
-------------------------

If you want to launch the entire software stack without starting xterms,
you can use the `nox_start.sh` script. The same approach will be taken with
delays between phases as above, but output will be directed to the ROS logs.
This is useful if you are logging in remotely with just a shell script
but no remote X11 server, or if you want to avoid a dependency on keeping
the xterms running.

To take this approach, issue the following commands. You can do this
remotely using a regular ssh session (without X11 tunnelling):
    
    roscd sawr_master
    ./nox_start.sh

Launching Phases Manually
-------------------------

Sometimes you want to test phases separately if you are doing debugging.
You can launch a phases yourself manually using

    roslaunch sawr_master init_X.launch

With X one of 0, 1, 2, or 3.  If you want, launch these in separate terminals
or ssh sessions to see the output.  Note that you do not have to be in 
the ``sawr_master`` directory.

Note: all phases need to share a single `roscore` instance.  You can start
this separately, but it is also started automatically by `roslaunch` if
it is not already running.  The above scripts start it automatically.

Launch Everything at Once
-------------------------

This launches all the phases at the same time without delays in between.
For the reasons noted above, this might not work (it is known not to under
ROS Indigo) but (if certain issues in ROS or the nodes are resolved over time)
may work on your installation. If you want to, you can try simultaneous launch
using:

    roslaunch sawr_master init.launch

Note this does not use a script but just ROS tools.  You do not have to be in
the `sawr_master` directory.

Teleop and Visualization
------------------------
Now that the software stack is launched, you can "drive" the robot using a
teleoperator and visualize the progress of SLAM (eg generation of a map) with
rviz.

Do the following in two separate windows:

    roslaunch sawr_navigation display.launch
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py

You can also launch these using scripts:

    roscd sawr_master
    ./scripts/viz.sh &
    ./scripts/teleop.sh

Manual Navigation
-----------------
Once you have a teleop running and a visualization, you can drive the robot
around manually and build up a map.  You can observe the map being updated
dynamically in rviz. Once you have a good map for a particular environment
you can [save it](http://wiki.ros.org/map_server) using the `map_saver` 
command to the map server, and then using the `start_saved.sh` script
provided, restart the system with that map. Upon restart with a saved map,
the robot will immediately try to localize itself against the map to
determine its initial position.

Autonomous Navigation
---------------------
Once you have a map and the robot has localized itself against the map,
you can try autonomous navigation. This is done by specifying a goal position
and orientation. Technically this is done by sending a message to the 
`move-base` sub-system which initiates a ROS navigation "action" (an "action"
is long-running process in ROS that you can observe the progress of, cancel, 
etc.). Actions are particularly appropriate for navigation, but also useful
in many other kinds of autonomous tasks.

It is possible to initiate a navigation action using rviz. Once you have a map
displayed and the robot has localized, select the "Set Goal" button at the top
of rviz, then click on a point on the map, drag a bit to select the 
orientation, and release the button. Rviz will then post the goal, and,
if all goes well... your robot will autonomously navigate to the goal 
position and orientation, avoiding obstacles on the way.

Remote Access
-------------
The above can be done using a HDMI cable and keyboard connected to the robot
but this is obviously not very satisfactory.  Instead we want to be able to
control the robot remotely and have it run untethered.
 
If you set up remote access via ssh as noted in [INSTALL.md](INSTALL.md),
you will be able to launch the software stack remotely.  If you use 
`ssh -X` to access the system over the internet, even the X11 versions with 
multiple terminal windows will work---definitely on a remote Linux or MacOS 
system but even from Windows if you install an X server such as 
[Xming](http://www.straightrunning.com/XmingNotes/).

Unfortunately rviz will not work remotely over X11, so to use it remotely
you will have to either set up remote ROS access (which will be insecure
unless you also set up a VPN like OpenVPN) or set up a VNC server on your
robot (less efficient, but VNC can be tunnelled over ssh).

You can also give goal states for navigation using command line tools, but
you would have to figure out the coordinate system the robot uses for its map.

See the security discussion in the [INSTALL.md](INSTALL.md) file to set up
remote access.  Doing this securely on a public network is unfortunately
difficult so you may want to start with a private WiFi network (eg behind an
external firewall).

Launching Automatically at Boot
-------------------------------
The simplest way to launch the system automatically at boot is to enable
automatic login and then configure it as a 
[Startup Application](https://help.ubuntu.com/16.04/ubuntu-help/startup-applications.html).
Note that you may have to run it inside a terminal window to get this to work.
This is somewhat insecure, as someone could in theory plug in a keyboard, 
mouse, and monitor to your robot and access your account, but you can always
have your login time out quickly.  However, since anyone with physical access
to your robot can typically break in without too much trouble anyway even if
you don't set up automatic login...

You can also configure a [script to run at boot using systemd](https://linuxconfig.org/how-to-automatically-execute-shell-script-at-startup-boot-on-systemd-linux).
This is a bit more robust and will work even without automatic login, and 
even if you disable the windowing system, but is a little harder to set up.
