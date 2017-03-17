Follow these instructions to set up the software stack on your robot.
Once set up,
refer to [LAUNCH.md](LAUNCH.md) for instructions on how to use the stack.

# Install Ubuntu 16.04

The SW stack is based on [ROS Kinetic](http://wiki.ros.org/kinetic)
which works best under [Ubuntu 16.04](http://releases.ubuntu.com/16.04/).
Both the 
[Intel&reg; Joule&trade;](https://www.google.com/webhp?sourceid=chrome-instant&ion=1&espv=2&ie=UTF-8#q=intel+joule&*)
and the 
[UP Board from Aaeon](http://www.up-board.org/up/) 
support desktop versions of Ubuntu 16.04.

**NOTE:** During setup,
it is highly recommended that you do NOT use a username or password
that are used elsewhere,
and also you should NOT store personal information 
(including web browser caches or web service account credentials) on the robot.
As noted later, among other issues remote access to ROS can be insecure.

## UP Board (included with the Intel Realsense Robotic Development Kit)

Follow instructions at
[https://up-community.org/wiki/Ubuntu](https://up-community.org/wiki/Ubuntu)
to first set up a standard distribution of Ubuntu 16.04.

In summary, first
[download an ISO image for Ubuntu 16.04](https://www.ubuntu.com/download/desktop)
and load it onto a bootable USB stick using, for instance,
[Rufus](https://www.ubuntu.com/download/desktop/create-a-usb-stick-on-windows)
or, if you have Ubuntu running already on another computer, the
[Startup Disk Creator](https://www.ubuntu.com/download/desktop/create-a-usb-stick-on-ubuntu).
Note: it is possible and perhaps preferrable to install a lightweight variant
such as Xubuntu or Lubuntu.
But the normal distribution works fine and generally is more consistent with
existing documentation.
Ubuntu Core has not been tested with this release.

Then boot the UP Board and tap the DEL key to get into the Bios (if it
prompts for a password just press enter).
Go to the Boot menu and make the
first boot option be the USB drive, then press F4 to save and boot.

If you can be online during the boot process things will go smoother.
The simplest way to do this is to use a wired connection, since you
might have to install drivers for your WiFi dongle in a later step.

After you have installed Ubuntu,
install (if you also want access to board features like GPIOs, I2C, etc)
the special kernel for the UP board, using basically the following
sequence:

    sudo add-apt-repository ppa:ubilinux/up
    sudo apt update
    sudo apt install linux-upboard
 
As of this writing the instructions for modifying GRUB to boot
the UP kernel by default are missing a few things.
In particular,
to get it bring up the GRUB menu on boot and to "save" the last kernel booted,
you need to ensure the following two lines are
in ``/etc/default/grub`` (using ``sudoedit``):

    GRUB_DEFAULT=saved
    GRUB_SAVEDEFAULT=true

and also ensure that the following line is commented out:

    # GRUB_HIDDEN_TIMEOUT=0

After making these changes to ``/etc/default/grub``, run

    sudo update-grub

After selecting the ``*-upboard`` kernel manually
at least once during boot using the "Advanced" options,
reboot /without/ selecting a kernel manually and
confirm that what ``uname -r`` produces still
has ``upboard`` in the name.

The UP kernel,
in addition to providing access to board features,
already includes the RealSense uvc patches, saving us a step later.

## Intel Joule 

There is a special install of Ubuntu 16.04 for Joule available from
[Canonical](https://developer.ubuntu.com/core/get-started/intel-joule#alternative-install:-ubuntu-desktop-16.04-lts).

Note that for ROS,
currently you want to install "Desktop Ubuntu" rather than "Ubuntu Core".
You probably also need to update the Joule BIOS to the latest version first as indicated.
Updating the BIOS is relatively straightforward but you will
need a Windows computer.

This version of Ubuntu also provides access to GPIOs, etc.
The Realsense drivers are also already included,
so we can skip the uvc patch later.

# Open a Terminal Window

You will need command line access to do most things.
Click on the topmost icon in the left menu bar with the Ubuntu logo,
type "Terminal" in the search field,
then drag the terminal icon to the menu bar for later use.

Click it and open a terminal.
To open additional terminals you need to right click.

# Install Basic Development Tools

Enter the following to install git and basic development tools:

    sudo apt-get install git build-essential

# Install WiFi Drivers (UP Board Only)

Since the UP Board does not include WiFi,
you will need to find a WiFi dongle and get it working.
This will probably also involve finding and installing drivers.

The TP-LINK AC600 (Archer T2UH) is a reasonable choice, as it supports
both 2.4GHz and 5GHz bands and has a high-gain antenna.
Driver installation for this dongle looks like the following.
Note that (wired) network access is necessary...

    mkdir -p ~/Drivers/T2UH
    cd ~/Drivers/T2UH
    git clone https://github.com/Myria-de/mt7610u_wifi_sta_v3002_dpo_20130916.git
    cd mt7610u_wifi_sta_v3002_dpo_20130916
    make
    sudo make install
    sudo cp RT2870STA.dat /etc/Wireless/RT2870STA/RT2870STA.dat

Then reboot.
WiFi should come up automatically.
Log into your access point and test.

# Add User to dialout Group

Add the current user to the ``dialout`` group as follows:

    sudo usermod -a -G dialout $USER

Then log out and log back in again.
Check group membership using and confirm ``dialout`` is listed:

    groups

This group membership
is necessary to access the UART device for the 
[USB2AX](http://www.xevelabs.com/doku.php?id=product:usb2ax:quickstart)
to drive the motors.

# Install Dynamixel SDK

The [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK)
library supports communication with the MX-12W "smart motors" 
used in the SAWR.
Here is a summary of how to install it:

    mkdir -p ~/Drivers
    cd ~/Drivers
    git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
    cd DynamixelSDK/c++/build/linux64
    make
    sudo make install

You will also have to configure the left servo with ID 1 
(which in practice means you leave it alone, since that is the default)
and the right servo with ID 2.
You should also lower the communication speed to 115200bps for
increased reliability.
You can use the 
[Dynamixel Wizard](http://support.robotis.com/en/software/roboplus/dynamixel_monitor.htm)
(yes, it does work with the USB2AX)
from Windows although there is
now some support for doing this from ROS as well.
The default ID and communication rate settings can also be
reconfigured in [sawr_base/src/sawr_base.cpp](sawr_base/src/sawr_base.cpp) if you
have to change them.

# Install librealsense

The [librealsense SDK](https://github.com/IntelRealSense/librealsense)
supports the Intel RealSense cameras.
The SAWR can use either the R200 or the ZR300 but the 
R200 is recommended for use with the UP Board and the ZR300 for use with the 
Joule.
Here is a summary of how to install:

    sudo apt-get install libusb-1.0-0-dev pkg-config libglfw3-dev cmake
    mkdir -p ~/Drivers
    cd ~/Drivers
    git clone https://github.com/IntelRealSense/librealsense.git
    cd librealsense

If you are on a Joule and want to use an R200,
open ``src/ds-device.cpp`` with an editor.
Search for ``PRESET_BEST``.
On these 9 lines, change all occurences
of 60 in the fifth element of each initializer to 30.
Save.
This changes the default frame rate to 30Hz which is necessary for
the stability of the sample programs on the Joule when using an R200.
However, if you want to use a ZR300 with an UP Board,
the drivers included in the provided kernel do not work at the time
this was written.
You will have to patch a standard distribution with
a more up-to-date kernel 
(which would also mean you can't use GPIOs, etc on the UP).

Install the librealsense user-space SDK libraries as follows:

    mkdir build
    cd build
    cmake ../ -DBUILD_EXAMPLES=true
    make
    sudo make install

Make sure the camera is not plugged in, then run

    cd ..
    sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules && udevadm trigger

Note that, except in the case of a ZR300 on an UP Board as noted above,
you do NOT need to run the kernel patch script.
Plug in your camera, and check that it is recognized with

    sudo dmesg | tail -n 50

Finally, run a test such as

    cpp-capture

Which should display the color, depth, and IR channels from the camera.

# Install ROS Kinetic

Follow the 
[online install instructions for ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
including sourcing the ROS setup script in your ``.bashrc``.
A summary follows:

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    sudo apt-get update
    sudo apt-get install ros-kinetic-desktop-full
    sudo rosdep init
    rosdep update
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    sudo apt-get install python-rosinstall

Note this downloads about 500MB of data and can take an hour or more,
depending on your network connection.

# Set Up a Catkin Workspace 

Follow [the Catkin workspace tutorial](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
to configure a Catkin workspace, 
a summary of which follows:

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    catkin_init_workspace
    cd ~/catkin_ws/
    catkin_make

Add this workspace to your ``.bashrc`` as well:

    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc

When you start a new shell from now on your environment will have been set up
automatically.

# Install SAWR Package

If you have not done it already,
the SAWR software itself can be copied from
the [SAWR github repository](https://github.org/01org/sawr) as follows:

    cd ~/catkin_ws/src
    git clone https://github.org/01org/sawr 

# Install Extra ROS Dependencies

The SAWR package depends on a few other ROS packages.
These can be installed with ``apt-get`` as follows:

    sudo apt-get install ros-kinetic-dynamixel-sdk \
                         ros-kinetic-realsense-camera \
                         ros-kinetic-depthimage-to-laserscan \
                         ros-kinetic-gmapping \
                         ros-kinetic-move-base \
                         ros-kinetic-navigation \
                         ros-kinetic-teleop-twist-keyboard 

# Compile SAWR Package

Prepare code in the SAWR package for execution:

    cd ~/catkin_ws
    catkin_make

# Remote Access and Security

By default an ssh server is not installed on Ubuntu and there is also no firewall.
You can install an ssh server and client using

    sudo apt-get install openssh-server openssh-client
    
and following the 
[instructions here]() to configure it.
Note that for enhanced security, after installation you should set up
a public key, copy it to the computer you intend to access the robot with,
then disable remote logins via password.
See the references at the bottom of the above page.

You should also 
[set up a firewall:](https://www.digitalocean.com/community/tutorials/how-to-set-up-a-firewall-with-ufw-on-ubuntu-16-04).
If you want remote ssh, then you need to explictly allow it.
Once this is setup, you will be able to remotely and securely log into your robot
to start the SW stack and use keyboard teleop.
If you use ``ssh -X`` you will even be able to open windows.
Unfortunately remote graphics such as in ``rviz`` tends not to work.
You can also 
[set up a VNC and tunnel it over ssh](https://www.linode.com/docs/applications/remote-desktop/install-vnc-on-ubuntu-16-04).
If set up properly this can be reasonably secure and will allow graphical applications.

Note that if the firewall is running then 
[remote network access to ROS](http://wiki.ros.org/ROS/NetworkSetup)
will not be possible.
You can temporarily disable the firewall in this case using ``sudo ufw disable``,
and of course reenable it with ``sudo ufw enable``, but while the firewall is 
disabled your robot will be vulnerable and should not be on a public network.
ROS is simply not defined with security in mind and should not be opened to an outside network.
Unfortunately ssh tunneling will not work for remote ROS access as ROS uses
a number of dynamically allocated ports.

A more secure option for remote ROS access,
and recommended if you want to access your robot remotely over
the internet, is to 
[set up a VPN](http://wiki.ros.org/ROS/NetworkSetup).
Assuming your WiFi subsystem does not support running a VPN in hardware,
you probably will have to do this in software using something like
[OpenVPN](https://www.digitalocean.com/community/tutorials/how-to-set-up-an-openvpn-server-on-ubuntu-16-04).
This is complex to set up but can give you secure remote access.

We are currently investigating rosbridge and how to secure a remote web console but note that
the standard configuration of rosbridge opens a port that allows
anyone to access it using an insecure websocket.
For the time being, if you use this, it recommended to do so only on a secure and isolated network.

# Next Steps

The installation should now be ready to launch.
See [LAUNCH.md](LAUNCH.md) for further instructions.
