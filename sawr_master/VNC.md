TurboVNC
========
[TurboVNC][TurboVNC] is a high performance remote desktop system that also
enables remote 3D graphics. It is based on top of a system called VirtualGL
that supports off-screen 3D rendering via OpenGL pbuffers, tuned compression
mechanisms for 3D content, and a special high-performance JPEG compression
library.

Prerequisites
-------------

### Automatic WiFi Connection

You have to be able to connect via SSH even when not logged in. Unfortunately,
under Ubuntu 16.04, by default WiFi connections set up under a particular user
are dropped when that user logs out.

This can be fixed by editing the connection using the Network Manager (pull
down the network menu at the upper right after establishing a connection t
your WiFi network), selecting Edit on the appropriate connection, and selecting
"All users many connect to this network" under the "General" tab. Also make
sure the "Automatically connect to this network when it is available"
selection is checked.

### System Update

Make sure your system is up to date:
    sudo apt-get update
    sudo apt-get upgrade
    sudo apt-get dist-upgrade

### Ubuntu MATE (Optional)

Install the Ubuntu MATE Desktop environment for use with VNC to avoid the
overhead of Unity and various issues and bugs when using that environment with
TurboVNC. This is optional but will significantly improve performance.
    sudo apt-get install mate-desktop-environment 
    sudo apt-get install mate-dock-applet mate-desktop-environment-extra

This will take about 450MB of storage. After installation, log in using a
normal keyboard and display to initialize it. If you do a first login via VNC,
automatic initialization currently has some issues (the panel crashes) and you
may have to do some manual initialization.

Also, you may find when you log into Unity that the theme is a little messed
up: missing icons, strange colors, etc. You can correct this by re-selecting a
theme, such as "Ambient", under the Appearance tab in System Settings.

You can remove the Unity environment afterwards if you wish to get some space
back. BEFORE removing Unity, though, login to MATE and ensure that it is 
configured and that you are comfortable with it.

Here is how you can remove Unity:
    sudo apt-get purge unity*
    sudo apt-get autoremove

If you want to reinstall Unity later, you can use:
    sudo apt-get install unity
 
If you chose not to install MATE, you can still run TurboVNC with Unity
but you need to use the `-3dwm` option when starting the server, as Unity
depends on OpenGL to generate various bits of (unnecessary) eye candy.

If you want to remove MATE (again, make sure you have at least one other
display manager installed and working...) use
    sudo apt-get purge mate-desktop-environment-core
    sudo apt-get purge mate-desktop-environment-extra
    sudo apt-get autoremove

Setup
-----
Follow the [instructions to set up a TurboVNC server][TurboVNC_UG],
a summary of which, appropriate for Ubuntu 16.04, is given below. 

Download 
[`libjpeg-turbo-official_1.5.1_amd64.deb`][TurboJPEG_deb],
[`virtualgl_2.5.2_amd64.deb`][VGL_deb], and 
[`turbovnc_2.1.1_amd64.deb`][TurboVNC_deb] from the corresponding
SourceForge download sites. Later versions should also work.

Install these packages as follows; note you need to remove any previous 
installation of VirtualGL first. It's ok to run the removal command to
check even if there isn't a previous installation.
    sudo dpkg -i libjpeg-turbo-official_1.5.1_amd64.deb
    sudo dpkg -r virtualgl
    sudo dpkg -i virtualgl_2.5.2_amd64.deb
    sudo dpkg -i turbovnc_2.1.1_amd64.deb

You may want to add `/opt/TurboVNC/bin` to your path. If so, add the
following to your `.bashrc`:
    export PATH=/opt/TurboVNC/bin:$PATH
The following doesn't assume you've done this, though.

After doing the above, log out, and log back in remotely using SSH 
    ssh {user}@{host}

Temporarily shut down the display manager:
    sudo service lightdm stop
If you still have a screen attached to the target computer, it should go
black at this point.

Run the VirtualGL configuration program:
    sudo vglserver_config
select option 1, accept all the suggested configuration defaults, then
use X to exit.

Restart the display manager:
    sudo service lightdm start

Add any user who will use TurboVNC to the `vglusers` group.  In the 
following we assume there is only going to be one TurboVNC user, 
and that this is the current user:
    sudo usermod -a -G vglusers $USER

Also add root to the `vglusers` group:
    sudo usermod -a -G vglusers root

Log out and log back in and confirm that you are part of the `vglusers`
group using
    groups

Manually Starting TurboVNC
--------------------------
Login remotely to the computer using SSH. Replace `{host}` with the name or
IP address of the target computer, and `{user}` with an appropriate username.
    ssh {user}@{host}

Start a TurboVNC server. After it starts, it will give you a display number to
use (typically `:1`).  In the following we will refer to this as `:{n}`.
    /opt/TurboVNC/bin/vncserver

NOTE: the above is for Ubuntu MATE.  If you are running the stock Unity 
display manager (_not_ recommended for performance reasons...) then you will
have to use the following instead:
    /opt/TurboVNC/bin/vncserver -3dwm
The _GOOD_ news of doing this though is that since everything is considered to
be 3D capable, you don't need to prefix applications with `vglrun`.  The bad
news is that everything will be considered 3D and will use more resources than
necessary.

If this is the first time you have used it, TurboVNC may prompt for an 8-character
password. Note that this provides only minimal security, and in particular the
datastream will be unencrypted. If you want better security, using an SSH
tunnel is recommended (see below).

Now back on the client machine, start a viewer on the appropriate host and
display number. If you are on a Linux machine in which you have also installed
TurboVNC, you can use
  /opt/TurboVNC/bin/vncviewer

The viewer may complain about not being able to find a Java runtime. If it
does, install OpenJDK-8 as follows (_not_ 9; TurboVNC does not currently support
version 9):
  sudo apt-get install openjdk-8-jdk

Once you start the viewer successfully it will open a window and prompt for a
server. Enter the hostname or IP, followed by a colon and the display number,
as follows: `{host}:{n}`. The viewer will then prompt for the password you set
in the previous step.

You should now see a desktop. However 3D applications, if run in the usual
way, will complain that there is no OpenGL support. You have to grant access to
OpenGL for each 3D application using the `vglrun` prefix command. For example,
suppose we want to run OpenSCAD, a 3D CAD program. We would need to use the
following:
  /opt/VirtualGL/bin/vglrun openscad
This command has [various options][VGLrun]. One nice thing about this though
is that all processes started by the target command are also covered. So for
instance, you can use `vglrun` on scripts that start other 3D things, or even
terminals that start things, or `rosrun` or `roslaunch` that launch other
programs that (eventually) start a process that actually needs 3D. Of course
this may start things that don't need 3D, so you should still be a _little_
selective.

Stopping a Manually Started VNC Server
--------------------------------------
After logging out, stop the TurboVNC server via the SSH shell to the remote
machine using
  /opt/TurboVNC/bin/vncserver -kill :{n}

If you want, you can also list all current sessions:
  /opt/TurboVNC/bin/vncserver -list

Using SSH Tunneling
-------------------
_NOTE: This part of the instructions is still under development. In particular,
TurboVNC has some built-in options for SSH tunnelling that may make setting up
your own tunnel unnecessary._

If you are on an insecure network (for instance, a wireless network) and want
improved security, you can set up SSH tunneling. You can set up a tunnel using
SSH as follows:
   ssh -L {localport}:localhost:{remoteport} {user}@{host}

For example, if the VNC server is running on display `:1`, as it usually the
case, then it will use port 5901. Use a different port locally to avoid any
conflicts with local VNC servers, such as 15901:
   ssh -L 15901:localhost:5901 {user}@{host}

Now follow the steps above to start the remote server. The SSH session to set
up the port forwarding can also be used to start the remote VNC server
manually.

The only difference is that now you need to start the local viewer as follows: 
   /opt/TurboVNC/bin/vncviewer localhost:1::15901
and it won't prompt for a server or display.

If you are using another VNC client, such as tightvnc or Remmina, you can use
`localhost:15901` whenever it prompts for the server.

Tuning Performance
------------------
You can adjust the quality settings in the preferences of the TurboVNC viewer
to improve performance. Lowering the quality setting improves both uses less
bandwidth and reduces computation time. Try 4X chromatic subsampling in
particular.

To improve network performance, try to put as many nodes as possible on a wired
network. For instance, even if you have a robot that _needs_ to be wireless,
connect the laptop or computer controlling it directly to the access point with
a wired connection if possible. This can significantly reduce latency.

If you are not using Ubuntu MATE, you can also improve performance by switching to
it. In testing, MATE used roughly half the CPU of Unity on a Joule.


[TurboJPEG_deb]: https://sourceforge.net/projects/libjpeg-turbo/files/1.5.1/libjpeg-turbo-official_1.5.1_amd64.deb/download
[VGL_deb]: https://sourceforge.net/projects/virtualgl/files/2.5.2/virtualgl_2.5.2_amd64.deb
[TurboVNC_deb]: https://sourceforge.net/projects/turbovnc/files/2.1.1/turbovnc_2.1.1_amd64.deb
[TurboVNC]: http://www.turbovnc.org/
[TurboVNC_UG]: http://www.virtualgl.org/vgldoc/2_1_1/
[VGLrun]: http://www.virtualgl.org/vgldoc/2_1_1/#Advanced_Configuration



