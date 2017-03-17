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
but the SW has not yet been updated; stay tuned.

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
Note that there are versions for both acrylic and acetal.
If you can find it, acetal (aka POM) is much tougher and highly recommended, and because it is
more flexible, allows for some additional features, like a compliant suspension.
The SW stack enables basic autonomous navigation and currently supports
building a 2D map with gmapper and then planning and executing a motion path
using move-base.

The software stack consists of ROS configuration files
and some simple ROS nodes for motor control.
For the most part, we have kept the stack as simple as possible.
Additional instructions for the SW stack are
located in the ``sawr`` package:
* [sawr_master/INSTALL.md](sawr_master/INSTALL.md) - How to install the SW.
* [sawr_master/LAUNCH.md](sawr_master/LAUNCH.md) - How to run the SW after installation.

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
* [sawr_console](sawr_console/README.md) - Web API and console.

# Additional Material
Please look for the SAWR project under
[01.org](https://01.org/sawr)
for links to additional material.
For example, the basic build instructions included in this repo,
while intended to make the repo self-contained,
do not include high-resolution images to keep the repo size down.
For the same reason this repo does not include videos of the SAWR in action.
We will make these and other resources available externally,
with a SAWR project under 01.org acting as a hub pointing to
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
Third-party patches and contributions are welcome.
We do require you to read and agree to the Contribution License Agreement below.

## Contribution License Agreement
The software for the Simple Autonomous Wheeled Robot (SAWR) project is developed
and distributed under a BSD-3 license,
and the hardware design is distributed under a CC-BY-4.0 license.
Details are given in [LICENSES.md](LICENSES.md).

By making a contribution to this project, I certify that:

(a) The contribution was created in whole or in part by me and I
    have the right to submit it under the open source license
    indicated in the file; or

(b) The contribution is based upon previous work that, to the best
    of my knowledge, is covered under an appropriate open source
    license and I have the right under that license to submit that
    work with modifications, whether created in whole or in part
    by me, under the same open source license (unless I am
    permitted to submit under a different license), as indicated
    in the file; or
 
(c) The contribution was provided directly to me by some other
    person who certified (a), (b) or (c) and I have not modified
    it.

(d) I understand and agree that this project and the contribution
    are public and that a record of the contribution (including all
    personal information I submit with it, including my sign-off) is
    maintained indefinitely and may be redistributed consistent with
    this project or the open source license(s) involved.

# Contribution Prerequisites

* Create a [GitHub account](https://github.com/join) if you do not already have one.
* Search the existing [GitHub Issues](../../../issues) to see if your contribution is already documented.
* Submit a [new GitHub Issue](../../../issues/new) if one does not exist.
* Complete the provided template with your system configuration.
* For Bug fixes, include directions on how to reproduce the problem.
* For new functionality or features, please describe in detail the requirements.
* [Create a Fork](../../../fork) of the repository on GitHub for your contribution.

# Submitting Changes

* Create a Topic branch for your work.
* Base the topic branch on the ROS development branch `<ros release>-devel`.
* Ensure changes follow the [ROS coding Style Guide](http://wiki.ros.org/StyleGuide)
* Each commit needs to be functional/compile by itself.
* Follow [Git Commit Guidelines](https://git-scm.com/book/ch5-2.html#Commit-Guidelines) regarding commit formatting.
* Check for unnecessary whitespace with `git diff --check` before committing.
* Rebase commits to squash where appropriate.
* Verify the Test cases found in the package 'test' directory (if it exists...) are passing.
* Changes with new functionality should include new test cases.
* Submit a [Pull Request](../../../compare) to the repository.
* Ensure that it is flagged as "Able to merge", if not you may need to rebase your Fork.
* List the Issues fixed by the Pull Request.

### Monitor Your Pull Request

* The maintainers attempt to review all Pull Requests in a timely manner; worse case is once per week.
* If maintainers request changes or additional information for your Pull Request, the expectation is the submitter replies within two weeks.
* Pull Requests may be closed without being merged if there is no submitter response after 3+ weeks.

# Contribution Resources

* [Create a GitHub account](https://github.com/join)
* [GitHub Pull Request documentation](https://help.github.com/articles/using-pull-requests)
* [ROS coding Style Guide](http://wiki.ros.org/StyleGuide)
* [Git Commit Guidelines](https://git-scm.com/book/ch5-2.html#Commit-Guidelines)
