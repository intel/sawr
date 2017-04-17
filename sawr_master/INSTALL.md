Simple Autonomous Wheeled Robot (SAWR)
======================================
Software Installation
---------------------
Follow these instructions to set up the software stack on your robot.
Once set up, refer to [LAUNCH.md](LAUNCH.md) for instructions on how to
run and use the software stack.

SAWR uses ROS (Robot Operating System, an open-source middleware framework for
robotics) but this is _not_ a tutorial on how to use ROS for mobile autonomous
robotics.  Please see the [ROS wiki](http://wiki.ros.org/) and also the many
excellent [books on ROS](http://wiki.ros.org/Books). Please also visit and
follow the [SAWR 01.org project page](https://01.org/sawr) for more
discussion, information, and resources for learning about robotics
using SAWR.

Install Ubuntu 16.04
--------------------
The SAWR software stack is based on [ROS Kinetic][ROS] which currently works
best under "desktop" [Ubuntu 16.04][Ubuntu]. Both the [Intel&reg; Joule&trade;][Joule]
and the [UP Board from Aaeon][UP] support desktop versions of Ubuntu 16.04.

**SECURITY NOTE:** During setup, it is highly recommended that you _DO NOT_ use
a username or password that are used elsewhere, and also you _DO NOT_ store
personal information (including web browser caches or web service account
credentials) on the robot. You may also want to use a separate account for
administration and run the SAWR software on an account on the robot on without
sudo access. As noted later, among other issues, remote access to ROS can be insecure
unless you take special effort to secure it.

### UP Board (included with the Intel Realsense Robotic Development Kit)
Follow instructions at
[https://up-community.org/wiki/Ubuntu](https://up-community.org/wiki/Ubuntu)
to first set up a standard distribution of Ubuntu 16.04 on your UP Board.

In summary, first
[download an ISO image for Ubuntu 16.04](https://www.ubuntu.com/download/desktop)
and load it onto a bootable USB stick using, for instance,
[Rufus](https://www.ubuntu.com/download/desktop/create-a-usb-stick-on-windows) 
or [Win32DiskImager](https://wiki.ubuntu.com/Win32DiskImager) under Windows,
or, if you have Ubuntu running already on another computer, the
[Startup Disk Creator](https://www.ubuntu.com/download/desktop/create-a-usb-stick-on-ubuntu).

**NOTE:** for performance reasons it is preferrable to install a lightweight variant
such as Xubuntu or Lubuntu. However, the normal Unity-based Ubuntu distribution
works fine and generally is more consistent with existing documentation, so try
that first... you can always reconfigure the window manager later if you want.
Ubuntu Core and Ubuntu Server have _not_ been tested with this release.

Then boot the UP Board and tap the DEL key to get into the BIOS (if it prompts
for a password just press enter). Go to the Boot menu and make the
first boot option be the USB drive, then press F4 to save and boot.

If you can be online during the boot process things will go smoother.
The simplest way to do this is to use a wired connection, since you
might have to install drivers for your WiFi dongle in a later step.

After you have installed Ubuntu, install the special kernel for the UP board,
using basically the following sequence:

    sudo add-apt-repository ppa:ubilinux/up
    sudo apt update
    sudo apt install linux-upboard
 
As of this writing the instructions for modifying GRUB to boot the UP kernel
by default are missing a few things. In particular, to get the system to bring
up the GRUB menu on boot and to "save" the last kernel booted, you need to 
ensure the following two lines are in `/etc/default/grub` (using `sudoedit`).
Add the second line if necessary.

    GRUB_DEFAULT=saved
    GRUB_SAVEDEFAULT=true

You should also ensure that the following line is commented out as shown:

    # GRUB_HIDDEN_TIMEOUT=0

After making these changes to `/etc/default/grub`, run

    sudo update-grub

After selecting the `*-upboard` kernel manually at least once during boot
using the "Advanced" options, reboot _without_ selecting a kernel manually
and confirm that what `uname -r` produces still has `upboard` in the name.

This special UP kernel, in addition to providing access to board features
like GPIOs, already includes the RealSense uvc patches, saving us a step later.

### Intel Joule 
There is a special install of Ubuntu 16.04 for Joule available from
[Canonical](https://developer.ubuntu.com/core/get-started/intel-joule#alternative-install:-ubuntu-desktop-16.04-lts).

**NOTE:** for ROS, currently you want to install "Desktop Ubuntu" rather than 
"Ubuntu Core". You probably also need to update the Joule BIOS to the latest
version first as indicated. Updating the BIOS is relatively straightforward
but you will need a Windows computer with USB-3.0 and the USB3.0-to-USB-C 
cable included in the Joule development kit.

This version of Ubuntu also provides access to GPIOs, etc. The Realsense 
drivers are also already included in the kernel, so we can skip the uvc patch
later.

Currently only standard desktop Ubuntu (running Unity) is supported, not
Xubuntu or Lubuntu. However, if you really want XFCE or another lightweight 
window manager, it is possible to install it later and have it run by default 
(or even no window manager, which may make even more sense...).  

However, it is recommended to first complete a working install before trying
these options.

Open a Terminal Window
----------------------
You will need command line access to do most things. Click on the topmost icon
in the left menu bar with the Ubuntu logo, type "Terminal" in the search field,
then drag the terminal icon to the menu bar for later use.

Click it and open a terminal. To open additional terminals you need to right
click and select "New Terminal".

Install Basic Development Tools
-------------------------------
Enter the following to install git and basic development tools:

    sudo apt-get install git build-essential

Install WiFi Drivers (UP Board Only)
------------------------------------
Since the UP Board does not include WiFi, if you want wireless access you will
need to find a WiFi dongle and get it working. This will probably also involve
finding and installing drivers.  A 5GHz (ac) WiFi dongle is highly desireable
in this context but tends to only be supported by newer dongles that may not
yet have support built into the Linux distribution.

The TP-LINK AC600 (Archer T2UH) is a reasonable choice, as it supports both
2.4GHz and 5GHz bands and has a high-gain antenna. Driver installation for
this dongle looks like the following.

Note that (wired) network access is necessary as part of the process.

    mkdir -p ~/Drivers/T2UH
    cd ~/Drivers/T2UH
    git clone https://github.com/Myria-de/mt7610u_wifi_sta_v3002_dpo_20130916.git
    cd mt7610u_wifi_sta_v3002_dpo_20130916
    make
    sudo make install
    sudo cp RT2870STA.dat /etc/Wireless/RT2870STA/RT2870STA.dat

Then unplug your wired network and reboot. WiFi should come up automatically.
Log into your access point and test.

Add User to dialout Group
-------------------------
Membership in the `dialout` group is necessary to access the UART device for 
the [USB2AX](http://www.xevelabs.com/doku.php?id=product:usb2ax:quickstart)
which is used in turn to control the motors.

Add the current user to the ``dialout`` group as follows:

    sudo usermod -a -G dialout $USER

Then log out and log back in again. Check group membership using and confirm
`dialout` is listed:

    groups

Install Dynamixel SDK
---------------------
The [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) library
supports communication with the MX-12W "smart motors" used in the SAWR.

In theory you _should_ be able to install the Dynamixel SDK by installing the
`ros-kinetic-dynamixel` package, which is done below. In practice you need
the `dxl_monitor` program (see below) to configure your servos,
and currently that is unfortunately only available with a manual installation.

Here is a summary of how to install the Dynamixel SDK manually:

    mkdir -p ~/Drivers
    cd ~/Drivers
    git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
    cd DynamixelSDK/c++/build/linux64
    make
    sudo make install

You will also have to configure the left servo with ID 1 (which in practice
means you leave it alone, since that is the default) and the right servo with
ID 2.

You can use the
[Dynamixel Wizard](http://support.robotis.com/en/software/roboplus/dynamixel_monitor.htm)
(yes, it does work with the USB2AX) from Windows to reconfigure the ID,
although it is also possible to do it directly from Linux.

To configure the servo IDs in Linux, go to your Dynamixel SDK install,
go to the `examples` directory, and compile the `dxl_monitor` program:

    cd ~/Drivers/DynamixelSDK/c++/example/dxl_monitor/linux64
    make
    
Then connect _just the **right** servo_ to the USB2AX.  Make sure it still has power,
however, since the USB2AX does not provide it.  This will require temporarily
reorganizing the servo cabling to disconnect and bypass the left servo.
The problem is that
by default all servos have the ID of 1, so having both on the bus at once will
lead to a conflict.  In order to change the ID of the right servo to a different
value we have to first make sure it is the only one on the bus.

In the above directory start the `dxl_monitor` program and do the following
(note that input is intermixed with output here):
```
./dxl_monitor --device /dev/ttyACM0

***********************************************************************
*                            DXL Monitor                              *
***********************************************************************

Succeeded to open the port!

 - Device Name : /dev/ttyACM0
 - Baudrate    : 1000000

[CMD] scan
 [ID:001] Model No : 00360                ... SUCCESS 
..........................................................................................................................................................................................................................................................

Scan Dynamixel Using Protocol 2.0
............................................................................................................................................................................................................................................................

[CMD] w1 1 3 2

 Success to write

[CMD] scan

Scan Dynamixel Using Protocol 1.0
.
 [ID:002] Model No : 00360                ... SUCCESS 
..........................................................................................................................................................................................................................................................

Scan Dynamixel Using Protocol 2.0
............................................................................................................................................................................................................................................................

[CMD] exit
```
Basically what the `w1 1 3 2` command does here is write, using
Dynamixel Protocol 1, on the servo with ID 1, the value 2 to register 3.
Each MX-12W servo has a
[set of registers](http://support.robotis.com/en/product/actuator/dynamixel/mx_series/mx-12w.htm).
Register 3 corresponds to the ID, so this changes its ID to 2.
When you rescan, you see the (only) servo now has a new ID of 002.

Now you can put the servo wiring back to normal, as the default ID of 1 is fine for the other servo.
If you want you can rescan to make sure both servos are visible.

Use `help` with `dxl_monitor` if you run into trouble.   The `reset1` command is
also useful as it resets the servo to its factory settings.

We use the factory default baud rate of 1,000,000 in our motor driver so there is no
need to change that.

Install librealsense
--------------------
The [librealsense SDK](https://github.com/IntelRealSense/librealsense)
library supports the Intel RealSense cameras. The SAWR can use either the R200
or the ZR300 but the R200 is recommended for use with the UP Board and the 
ZR300 for use with the Joule.

(You should be able to install librealsense using the ros-kinetic-librealsense
package, which is done below.  You can skip this step and return to it for
a manual install if that does not work.)

Here is a summary of how to install librealsense manually:

    sudo apt-get install libusb-1.0-0-dev pkg-config libglfw3-dev cmake
    mkdir -p ~/Drivers
    cd ~/Drivers
    git clone https://github.com/IntelRealSense/librealsense.git
    cd librealsense

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

Plug in your camera, and check that it is recognized with

    sudo dmesg | tail -n 50

Finally, run a test such as

    cpp-capture

Which should display the color, depth, and IR channels from the camera.

Install ROS Kinetic
-------------------
Follow the [online install instructions for ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
including sourcing the ROS setup script in your `.bashrc`.

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

Set Up a Catkin Workspace 
-------------------------
Follow [the Catkin workspace tutorial](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
to configure a Catkin workspace, a summary of which follows:

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    catkin_init_workspace
    cd ~/catkin_ws/
    catkin_make

Add this workspace to your `.bashrc` as well:

    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc

When you start a new shell from now on your environment will have been set up
automatically.

Install SAWR Packages
---------------------
If you have not done it already, the SAWR software itself can be copied from
the [SAWR github repository](https://github.com/01org/sawr) as follows:

    cd ~/catkin_ws/src
    git clone https://github.com/01org/sawr.git

Install Extra ROS Dependencies
------------------------------
The SAWR package depends on a few other ROS packages.
If necessary these can be installed with ``apt-get`` as follows:

    sudo apt-get install ros-kinetic-dynamixel-sdk \
                         ros-kinetic-librealsense \
                         ros-kinetic-realsense-camera \
                         ros-kinetic-depthimage-to-laserscan \
                         ros-kinetic-gmapping \
                         ros-kinetic-move-base \
                         ros-kinetic-navigation \
                         ros-kinetic-teleop-twist-keyboard 

This _should_ also automatically pull in the RealSense and Dynamixel
drivers, but if not, you can try the manual installations above.

Compile SAWR Package
--------------------
Prepare code in the SAWR package for execution:

    cd ~/catkin_ws
    catkin_make

Remote Access and Security
--------------------------
No system is ever perfectly secure, and there are tradeoffs between 
functionality and security.
Ubuntu is [fairly secure by default](https://insights.ubuntu.com/2016/12/08/ubuntu-16-04-lts-security-a-comprehensive-overview/)
but there are various ways to [improve it](https://www.thefanclub.co.za/how-to/how-secure-ubuntu-1604-lts-server-part-1-basics).
However, if you want to support remote access to your robot to teleoperate
it or to remotely visualize ROS data you will have to do some work.
The following are suggestions, not a complete solution for secure access.

### SSH
SSH provides secure remote access to a Linux system, and allows you to log
into your robot without having to hook up a display, keyboard and mouse.
This is obviously useful when the robot is roaming around.

However, by default an SSH server is not installed on Ubuntu. You can 
however easily install it using:

    sudo apt-get install openssh-server
    
Follow the [instructions here][SSH] to configure it.

Note that for enhanced security, after installation you should set up
a public key, copy it to the computer you intend to access the robot with,
then disable all remote logins via password.

It is recommended to leave your passphrase blank (it complicates automated
logins) but use a 4096-bit RSA key and use ufw to limit the rate of
login attempts to compensate.

### Firewall
You should also [set up a firewall][UFW]. With the firewall enabled, if you want 
remote ssh access, then you need to explictly allow it:

    sudo ufw enable
    sudo ufw allow ssh

Although, as noted above, to improve security you might want to limit the
rate of login attempts:

    sudo ufw limit ssh
    
Once this is setup, you will be able to remotely and securely log into your
robot to start the SW stack and use keyboard teleop. If you use `ssh -X` 
you will even be able to open windows.

Unfortunately remote graphics such as in `rviz` does not work (there is 
support in OpenGL for remote graphics, but...) and using ROS remotely may
require disabling the firewall.

Depending on what you want to do, you may also want to allow the ntp and http
protocols:

    sudo ufw allow ntp
    sudo ufw allow http

### Secure Tunnelled VNC
You can also 
[set up a VNC and tunnel it over ssh](https://www.linode.com/docs/applications/remote-desktop/install-vnc-on-ubuntu-16-04).
If set up properly this can be reasonably secure and will allow graphical
applications like rviz, although it will not be especially performant.

### Remote ROS Access
Note that if the firewall is running then 
[remote network access to ROS](http://wiki.ros.org/ROS/NetworkSetup)
will not be possible. You can temporarily disable the firewall in this case 
using 

    sudo ufw disable

and of course reenable it with

    sudo ufw enable  
    
Unfortunately, while the firewall is disabled your robot will be vulnerable
and should not connected directly to a public network.

ROS is not defined with security in mind and should not be opened to an 
outside network. Unfortunately ssh tunneling will not work for remote ROS
access as ROS uses a number of dynamically allocated ports.

### OpenVPN
A more secure option for remote ROS access, and recommended if you want to 
access your robot remotely over the internet, is to set up a VPN.

Assuming your router/WiFi access point does not support running a VPN
in hardware, you probably will have to do this in software using something
like [OpenVPN][OpenVPN]. This is complex to set up but can give you secure
remote access to ROS.

### ROSbridge: Future Work
We are currently investigating rosbridge and how to secure a remote web 
console with Robot Web Tools.  However, note that the standard configuration
of rosbridge opens a port that allows anyone to access it using an insecure
websocket, so it's just as insecure as plain ROS.

However, ROSbridge also does support _secure_ websockets, and so with the
right setup, it should be possible to build a reasonably secure web API using
it.

We are investigating this. However, for the time being if you use a default
rosbridge configuration it is also recommended to do so only on a secure and
isolated network.

Next Steps
----------
The installation should now be ready to launch.
See [LAUNCH.md](LAUNCH.md) for further instructions.

[ROS]: http://wiki.ros.org/kinetic
[Ubuntu]: http://releases.ubuntu.com/16.04/
[Joule]: https://software.intel.com/en-us/iot/hardware/joule
[UP]: http://www.up-board.org/up/
[SSH]: https://help.ubuntu.com/community/SSH/OpenSSH/Configuring
[UFW]: https://www.digitalocean.com/community/tutorials/how-to-set-up-a-firewall-with-ufw-on-ubuntu-16-04
[OpenVPN]: https://www.digitalocean.com/community/tutorials/how-to-set-up-an-openvpn-server-on-ubuntu-16-04
