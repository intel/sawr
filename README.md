# Simple Autonomous Wheeled Robot (SAWR)
The goal of the SAWR project is to define a basic "example robot"
to demonstrate autonomous navigation using ROS with
the Intel&reg; RealSense&trade; 3D cameras.
This robot can also be used as a software
development and testing platform and
to deliver training material.

This repository contains both the hardware specification and the software
to enable you to build and run your own SAWR.
For compute and sensing, you can
use either the Intel&reg; RealSense&trade; Robotic Development Kit
(which includes an UP Board and an Intel&reg; RealSense&trade; R200 3D camera)
or an Intel&reg; Joule&trade; compute module combined with
an Intel&reg; RealSense&trade; R200 3D camera. 
Note: The frame is designed to support a RealSense&trade; ZR300 3D camera as well
but the SW has not yet been fully updated and tested; stay tuned.

The hardware specification,
located in ``sawr_description/hardware``,
is intended to be sufficient to allow you to
build your own SAWR if you have access to a laser cutter.
The hardware specification is in the form of some OpenSCAD files,
a bill of materials,
and basic build instructions.
Laser-cutting files (given as PDF) 
are derived from the OpenSCAD master but are
included for convenience as well.
There are versions for both acrylic and acetal (aka POM, Delrin, etc).
If you can find it, acetal is much tougher than acrylic and highly recommended, and because it is
more flexible, allows for some additional features, like a compliant suspension.
However, if you must use acrylic make sure you cut with the acrylic verion of the frame.

Assembly details are in
[sawr_description/hardware/Assembly/ASSEMBLY.md](sawr_description/hardware/Assembly/ASSEMBLY.md).

The SAWR software stack enables basic autonomous navigation and currently supports
building a 2D map with gmapper and then planning and executing a motion path
using move-base.  It does not yet support out-of-the-box secure teleoperation,
however.  You will have to either run ROS over an isolated network or set up your
own secure remote teleoperation interface.

The software stack consists of ROS configuration files
and some simple ROS nodes for motor control.
For the most part, we have kept the stack as simple as possible.
Additional instructions for the SAWR software stack are
located in the ``sawr`` package:
* [sawr_master/INSTALL.md](sawr_master/INSTALL.md) - How to install the SAWR software stack.
* [sawr_master/LAUNCH.md](sawr_master/LAUNCH.md) - How to run the SAWR software stack after installation.

# Packages
Here is a list of packages included in this project and
a summary of what each one contains.
Please refer to the README.md files in each package for more information:
* [sawr_master](sawr_master/README.md) - Master package, installation instructions, and launch scripts.
* [sawr_description](sawr_description/README.md) - Hardware specification and build instructions, URDF files.
* [sawr_base](sawr_base/README.md) - Motor controller and hardware interfacing.
* [sawr_scan](sawr_scan/README.md) - Camera configuration.
* [sawr_mapping](sawr_mapping/README.md) - SLAM configuration.
* [sawr_navigation](sawr_navigation/README.md) - Move-base configuration.

# Additional Material
Please look for the SAWR project under
[01.org](https://01.org/sawr)
for links to additional material.
For example, the basic build instructions included in this package, 
while included to make the package self-contained,
do not include high-resolution images to keep the download size down.
For the same reason this package does not include videos of the SAWR in action.
We will make these and other resources available externally
with a [SAWR project under 01.org](https://01.org/sawr) 
acting as a hub pointing to
the available resources.

In particular, 
we hope to make kits available for purchase and link to them from 
the [01.org SAWR project page](https://01.org/sawr).
If you want to make and sell a kit, no problem!
The licenses used permit commercial use and derivatives;
see [LICENSES.md](LICENSES.md) for details of the licenses, 
and the package.xml files in each directory for information on which licenses apply to each component.
However, please do let us know if you are producing a kit so we can link to you from
the above site.

# Contributing to the SAWR Project
This project welcomes third-party code via GitHub pull requests. 
Please review the [Contribution License Agreement](CLA.md) 
located in the root of the repository. 
For each pull request submitted, 
please state your legal name and the text, 
"I agree to the terms of the 'Simple Autonomous Wheeled Robot' (SAWR) CLA."
