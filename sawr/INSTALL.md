Follow these instructions to set up the software stack on your robot.
Once set up, refer to [LAUNCH.md] for how to use the stack.

# Install Ubuntu 16.04

## UP Board (included with the Intel Realsense Robotic Development Kit)

Follow instructions at [https://up-community.org/wiki/Ubuntu] to
first set up a standard distribution of Ubuntu 16.04.

In summary, first download an ISO image for Ubuntu 16.04 from
[https://www.ubuntu.com/download/desktop]
and load it onto a bootable USB stick using, for instance, Rufus
[https://www.ubuntu.com/download/desktop/create-a-usb-stick-on-windows]
or, if you are on Linux already, the Startup Disk Creator:
[https://www.ubuntu.com/download/desktop/create-a-usb-stick-on-ubuntu]

Then boot the UP Board and tap the DEL key to get into the Bios (if it
prompts for a password just press enter).  Go to the Boot menu and make the
first boot option be the USB drive, then press F4 to save and boot.

If you can be online during the boot process things will go smoother.
The simplest way to do this is to use a wired connection, since you
might have to install drivers for your WiFi dongle in a later step.

After you have installed Ubuntu,
install (if you also want access to board features like GPIOs, I2C, etc)
the special kernel for the UP board, using basically the following:

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

and that the following line is commented out:

    # GRUB_HIDDEN_TIMEOUT=0

After making these changes to ``/etc/default/grub``, run

    sudo update-grub

After selecting the upboard kernel manually
at least once during boot using the "Advanced" options,
reboot /without/ selecting a kernel manually and
confirm that what ``uname -r`` produces still
has ``upboard`` in the name.

The UP kernel,
in addition to providing access to board features,
already includes the RealSense uvc patches, saving us a step later.

## Intel Joule 

There is a special install of Ubuntu 16.04 for Joule available from
the following location:
[https://developer.ubuntu.com/core/get-started/intel-joule#alternative-install:-ubuntu-desktop-16.04-lts].

Note that for ROS,
currently you want to install "Desktop Ubuntu" rather than "Ubuntu Core".
You probably also need to update the BIOS first as indicated.
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

# Install WiFi Drivers

Since the UP Board does not include WiFi,
you will need to find a WiFi dongle and get it working.
This will probably involve finding and installing drivers.

The TP-LINK AC600 (Archer T2UH) is a reasonable choice, as it supports
both 2.4GHz and 5GHz bands and has a high-gain antenna.
Driver installation for this dongle looks like the following.
Note that (wired) network access is assumed...

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
Conversely, if you want to use a ZR300 with an UP Board,
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

Note this downloads about 500MB of data and can take an hour or more.

# Set Up a Catkin Workspace 

Follow [the Catkin workspace tutorial](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) to configure a Catkin workspace, 
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

**TODO: CHANGE PATHS TO RELEASED GITHUB SITE!**
The SAWR software itself can be installed from
the [SAWR github repository](https://github.com/otcshare/ros-fetchbot.git)
as follows:

    cd ~/catkin_ws/src
    git clone https://github.com/otcshare/ros-fetchbot.git

# Install Extra ROS Dependencies

The SAWR package depends on a few other ROS packages.
These can be installed with ``apt-get`` as follows:

    sudo apt-get install ros-kinetic-dynamixel-sdk \
                         ros-kinetic-realsense-camera \
                         ros-kinetic-depthimage-to-laserscan \
                         ros-kinetic-gmapping \
                         ros-kinetic-navigation \
                         ros-kinetic-rosbridge-server \
                         ros-kinetic-teleop-twist-keyboard

# Install Node.js and Dependencies

Install [Node.js](https://nodejs.org), a server-side Javascript environment
we use to create a web console for the SAWR.
We also want to install ``npm``, which is used to download external modules
used by our web server implementation.

    sudo apt-get install nodejs npm

Many scripts that use Node.js expect to find it under the name ``node``.
Unfortunately, that name was already in use in Ubuntu by another 
(rarely-used) package, and the correct way to invoke it is now ``nodejs``.
To avoid a number of errors from software that has not yet been
updated, create a symbolic link as follows:

    sudo ln -s /usr/bin/nodejs /opt/ros/kinetic/bin/node

We suggest putting the link 
in ``/opt/ros/kinetic/bin`` rather than ``/usr/bin`` to avoid
clashes if you actually do install the ``node`` package via ``apt-get``.

Go to the ``sawr_console/app`` directory and install the Node.js modules used
by the web console:

    roscd sawr_console/app
    npm install 

**TODO: One day this should be updated so the NPM install step
is handled by catkin_make.**

# Compile SAWR Package

Prepare code in the SAWR package for execution:

    cd ~/catkin_ws
    catkin_make

# Next Steps

The installation should now be ready to launch.
See [LAUNCH.md](LAUNCH.md) for further instructions.
