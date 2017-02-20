# Install Ubuntu 16.04

## UP Board (included with the Intel Realsense Robotic Development Kit)

Follow instructions at [https://up-community.org/wiki/Ubuntu]

First install a standard distribution of Ubuntu 16.04.  
Download an ISO image for Ubuntu 16.04 from
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
 
As of this writing the instructions for modifying GRUB to boot the UP kernel by default are missing a few things.
You need to ensure the following three lines are in /etc/default/grub (using sudoedit) 
to get it bring up the GRUB menu on boot and to "save" the last kernel booted:

GRUB_DEFAULT=saved
GRUB_SAVEDEFAULT=true
# GRUB_HIDDEN_TIMEOUT=0

After making these changes to /etc/default/grub, run
  sudo update-grub

After selecting the upboard kernel manually 
at least once during boot using the "Advanced" options, 
reboot /without/ selecting a kernel manually and 
confirm that "uname -r" still produces has "upboard" in the name.

The UP kernel, 
in addition to providing access to board features,
already includes the RealSense uvc patches, saving us a step later.

## Intel Joule 

There is a special install of Ubuntu 16.04 for Joule available from the following location:
https://developer.ubuntu.com/core/get-started/intel-joule#alternative-install:-ubuntu-desktop-16.04-lts

Note that for ROS,
currently you want to install "Desktop Ubuntu" rather than "Ubuntu Core".
You probably also need to update the BIOS first as indicated.   
Updating the BIOS is relatively straightforward but you will need a Windows computer.

This version of Ubuntu also provides access to GPIOs, etc.
The Realsense drivers are also already included,
so we can skip the uvc patch later.

# Open a Terminal Window

You will need command line access to do most things.   
Click on the topmost icon in the left menu bar with the Ubuntu logo, 
type "Terminal" in the search field,
then drag the terminal icon to the menu bar for later use.

Click it and open a terminal.   To open additional terminals you need to right click.

# Install Basic Development Tools

Enter the following to install git and basic development tools:
  sudo apt-get install git build-essential

# Install WiFi Drivers

Since the UP Board does not include WiFi, you will need to find a WiFi dongle and get it working.
This will probably involve finding and installing drivers.

The TP-LINK AC600 (Archer T2UH) is a reasonable choice.
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
WiFi should come up automatically.  Log into your access point.

# Add User to dialout Group

  sudo usermod -a -G dialout $USER

Then log out and log back in again.   Check group membership using

  groups

This is necessary to access the UART device for the USB2AX to drive the motors.
See [http://www.xevelabs.com/doku.php?id=product:usb2ax:quickstart]

# Install Dynamixel SDK

See [https://github.com/ROBOTIS-GIT/DynamixelSDK]
Here is a summary of how to install:
  mkdir -p ~/Drivers
  cd ~/Drivers
  git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
  cd DynamixelSDK/c++/build/linux64
  make
  sudo make install

# Install librealsense

See [https://github.com/IntelRealSense/librealsense]
Here is a summary of how to install:
  sudo apt-get install libusb-1.0-0-dev pkg-config libglfw3-dev cmake
  mkdir -p ~/Drivers
  cd ~/Drivers
  git clone https://github.com/IntelRealSense/librealsense.git
  cd librealsense
If you are on a Joule, open src/ds-device.cpp with an editor.  
Search for "PRESET_BEST".   On these 9 lines, change all occurences 
of 60 in the fifth element of each initializer to 30.  Save.
This changes the default frame rate to 30Hz which is necessary for 
the stability of the sample programs on the Joule.
  mkdir build
  cd build
  cmake ../ -DBUILD_EXAMPLES=true
  make
  sudo make install
Make sure the camera is not plugged in, then run
  cd ..
  sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules && udevadm trigger
Note that you do NOT need to run the kernel patch script.
Plug in the camera, and check that it is recognized with
  sudo dmesg | tail -n 50
Finally, run a test such as 
  cpp-capture

# Install ROS Kinetic

Follow the instructions at
  [http://wiki.ros.org/kinetic/Installation/Ubuntu]
including sourcing the ROS setup script in your .bashrc:
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
  sudo apt-get update
  sudo apt-get install ros-kinetic-desktop-full
  sudo rosdep init
  rosdep update
  echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  sudo apt-get install python-rosinstall

# Set up a Catkin Workspace 

Follow [http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment], summary follows:
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/src
  catkin_init_workspace
  cd ~/catkin_ws/
  catkin_make
Add it to your .bashrc
  echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
  source ~/.bashrc

# Install Extra ROS Dependencies

The SAWR package uses a few other ROS packages:
  sudo apt-get install ros-kinetic-dynamixel-sdk

# Install SAWR Package

TODO, FIX: Update with release github path and/or package name!!
  cd ~/catkin_ws/src
  git clone https://github.com/otcshare/ros-fetchbot.git

# Compile

  cd ~/catkin_ws
  catkin_make

