# Simple Autonomous Wheeled Robot (SAWR) Assembly Instructions

The SAWR is a simple learning platform for autonomous robotics. Despite its small size and low cost, it is capable of performing advanced mobile robotics tasks, including simultaneous localization, mapping, and navigation. 

The purpose of the SAWR is to provide a starting point for people who would like to learn more about advanced mobile robotics, in particular autonomous navigation and ROS, the Robot Operating System.

This simple robot (and its software stack) can also serve as a starting point for those seeking to develop more sophisticated robots based on the Intel&reg; RealSense&trade; RDK.

All frame components for the SAWR are parametrically designed with [OpenSCAD](http://www.openscad.org/) and can be easily adjusted to 
accommodate geometry of new electronic components if required. In addition to the Intel&reg; RealSense&trade; RDK, the frame also already has mounting points for the Intel&reg; Joule&trade; development kit and the ZR300 Intel&reg; RealSense&trade; 3D camera.

The SAWR has a simple design and uses a minimal number of easily available off-the-shelf parts, making this platform easy to assemble. This allows you to and move faster into the ROS development phase, rather than spending time on mechanical assembly and custom electronic components. In particular, the SAWR's locomotion is based on two Dynamixel MX-12W servos, which provide accurate odometry feedback and speed control, and are well-supported by ROS and the robotics community.

This document outlines all mechanical/electronic components and assembly steps of the SAWR. Images have been included in the SAWR package to go with this file but links are also provided to high-resolution files and even 3D models as appropriate.

## Mechanical Assembly and Electrical Wiring

![image002.png](images/image002.png)

All mechanical drawings, 3D assemblies, and electrical schematics presented in this document are also available in the form of web
links to CAD cloud storage. In general, the online versions of these images and models may be more up-to-date than the images in this document, in addition to in many cases allowing interaction, such as rotating the 3D model. Therefore, please follow the links in any case where a clearer view is needed or to ensure you have the latest version of the figure. You can also use OpenSCAD to look at the model locally.

All Fastener specifications and part numbers are provided in the BOM and 2D drawings. Part numbers are from the [McMaster-Carr catalog](http://www.mcmaster.com/).

All fasteners are sized in metric. In many cases equivalent parts can be substituted, so in addition to McMaster-Carr part numbers we also provide (in BOM and drawings) generic specifications of each part so you can find appropriate substitute parts if necessary.

Generally a 10% tolerance is permissable for holes and slots. One exception however where an accurate size is needed is for the bearings and the bolt used for the wheel axle. These must be exact so get metric bolts and bearings or edit the CAD files and/or cut files.

Frame, driver and wheel components are laser cut from two 450mm x 300mm (18x12 in) Acrylic or Acetal sheets: one 2mm thick and the second 3mm thick. Acrylic is perfectly fine and is low-cost, but Acetal (also known as POM, Delrin, or Duracon) is more durable and is recommended for heavy use, such as in a classroom, or if you need to transport the robot frequently. Using Acrylic for the 2mm sheet and Acetal for the 3mm sheet is also possible and gives most of the benefit of using Acetal alone.

If you just want to laser-cut the file, start with the PDF file, being careful not to resize it. If you do not have access to the size
of sheet specified, you can rearrange the parts using a 2D vector graphics editor such as [Inkscape](https://inkscape.org/), which is free. In this case, you want to start from the SVG file. If you need to rearrange the file with Adobe Illustrator, however, start from the PDF file, as Adobe Illustrator tends to resize SVG on import. In addition, the lines in these files have been colored red and blue.  Red lines should be cut first, followed by the blue lines (Trotec conventions). Depending on your laser cutter (i.e. if you have a Universal laser cutter), you may have to modify the colors to get the right cutting order. Depending on the laser cutter model the cut time is between 24 min to 1 hr.

### Sheet 1:450X300x2 mm
(18x12x0.078 in) -- 1 each.

Select Download from the following web pages
  * [Download Laser Cut file for 2mm sheet in SVG format](http://a360.co/2aKxAYn)
  * [Download Laser Cut file for 2mm sheet in PDF format](http://a360.co/2iRK3wz)

### Sheet 2:450x300x3mm
(18X12x0.118in) -- 1 each.

  * [Download Laser Cut file for 3mm sheet in SVG format](http://a360.co/2iRDmuA)
  * [Download Laser Cut file for 3mm sheet in PDF format](http://a360.co/2iRBB0e)

Here is [a drawing](http://a360.co/2a88Qpd) of all laser cut components with the names we will be using to refer to them in the assembly notes.

The Tower plate provides the option to mount either the RDK kit (UP board and Intel RealSense RS200) or an Intel Joule development kit with an Intel RealSense ZR300 camera. Itfs also possible to combine a Joule with an R200 camera.

![image006.jpg]([images/image006.jpg)

Here is a Bill of Materials (BOM), with estimated cost of parts. The links are for reference only; you may be able to find alternative suppliers for these parts.

  * [Download BOM (Bill of Materials) in XLS format](http://a360.co/2c8y1bV)
  * [Download BOM (Bill of Materials) in CSV format](http://a360.co/2c8xRBg)

## Wheels Assembly

![image007.png](images/image007.png)
![image008.jpg](images/image008.jpg)

_Drawing 1_

  * [Drawing 1](http://a360.co/29UsC8R)
  * [Wheel 3D Assembly]("http://a360.co/2agnle2)

Wheels are assembled from five layers of laser cut components. The outer  profile of the wheel has a "U" shape to capture an O-ring tire, which is also used as a belt drive.  The center layer of each wheel is 3mm thick, and all other layers are 2mm thick. The total wheel thickness is 11mm.

  * Press in two bearings into the center holes of two pairs of the outer 2mm 
    wheel layers. This is easiest before bolting the wheel layers together as 
    you can press in the bearing using a flat surface.
  * Secure all 5 layers of the wheel with 12 ea. M3 lock-nuts and M3x14
    screws. The central layerfs hole is slightly smaller than the outer 
    diameter of the bearing, so the bearings will not pass through the center 
    layer.
  * Repeat for the second wheel.

## Frame and assembly of other components

It is advisable to assemble the rest of the robot in the order given below. Note the location of the holes on the top of the tower: the orientation of the tower plate matters, it is not reversible!

  1. Attach components to the Tower plate in the following order: 
     (Drawings 2 A, B, C) 
     [Tower 3D Assembly](http://a360.co/2ago66G)
     ![image009.png]([images/image009.png)
     ![image011.png]([images/image011.png)
     Drawing 2A

  * Attach two brackets to the tower. For each bracket, install
     6ea. M2 hex nuts (provided in servo box) in the bracket and use 6ea.
     M2x8 screws to secure bracket to the Tower. Repeat for the
     second bracket.

       [Drawing2A](http://a360.co/2covpJj)

  * Insert M2 nuts into the servos, both along the outer faces (8 ea.) and 
    along the end where the bracket goes (4 ea.). We will only be
     using the latter for now but it is much easier to install the nuts in the
     servos before mounting them on the robot than afterwards.

  * Attach two servos to the brackets. For each servo, use 4 ea. M2x8
    screws and 4 ea. M2 hex nuts (nuts provided with servo kits)                    [Drawing 2A](http://a360.co/2covpJj)Drawing 2A

    ![image013.png](images/image013.png)
    ![image015.png]((images/image015.png)
    ![image016.jpg]((images/image016.png)
    Drawing 2B

  * Install the DFRobot DC-DC Power Convertor.  The power convertor goes on 
    the opposite side of the tower from the servos. Note the orientation of 
    the connectors; when viewed from the side the power converted is mounted
    on, the 5V output should be on the left. When viewed from thegbackh,
    where the servos are, the 5V output will be on the right.
    Use 4ea. M3x6mm spacers, 4ea. M3 locknuts and 4 ea. M3x14 screws/
    Refer to [Drawing 2B](http://a360.co/2coACk9).

  * Install standoffs for the UP board, but do not install UP
     board yet.&nbsp;We will install UP board last after we organize the
     wiring, much of which will run underneath the UP board. Use 4ea M2.5x16 mm
     standoffs and 4ea. M2.5x6 screws. Refer to </span><a
     href="http://a360.co/2coACk9"><span class=InternetLink><b><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman"'>Drawing 2B</span></b></span></a></li>
</ul>

<p class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
margin-left:36.0pt;text-align:justify;text-justify:inter-ideograph;line-height:
15.75pt;page-break-after:avoid'><a href="http://a360.co/2coACk9"><span
style='color:windowtext;mso-fareast-language:JA;mso-no-proof:yes;text-decoration:
none;text-underline:none'><!--[if gte vml 1]><v:shape id="Picture_x0020_1"
 o:spid="_x0000_i1039" type="#_x0000_t75" style='width:202.5pt;height:231pt;
 visibility:visible;mso-wrap-style:square'>
 <v:imagedata src="INSTRUCTIONS_files/image017.png" o:title=""/>
</v:shape><![endif]--><![if !vml]><img border=0 width=270 height=308
src="INSTRUCTIONS_files/image018.jpg" v:shapes="Picture_x0020_1"><![endif]></span></a><span
style='mso-fareast-language:JA;mso-no-proof:yes'><!--[if gte vml 1]><v:shape
 id="Picture_x0020_10" o:spid="_x0000_i1038" type="#_x0000_t75" style='width:219pt;
 height:212.25pt;visibility:visible;mso-wrap-style:square'>
 <v:imagedata src="INSTRUCTIONS_files/image019.png" o:title=""/>
</v:shape><![endif]--><![if !vml]><img border=0 width=292 height=283
src="INSTRUCTIONS_files/image020.jpg" v:shapes="Picture_x0020_10"><![endif]></span></p>

<p class=MsoCaption>Drawing 2C </p>

<p class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
margin-left:36.0pt;line-height:15.75pt'><span style='font-size:10.5pt;
font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman";
color:#272727'><o:p>&nbsp;</o:p></span></p>

<ul type=disc>
 <li class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
     text-align:justify;text-justify:inter-ideograph;line-height:15.75pt;
     mso-list:l9 level1 lfo5;tab-stops:list 36.0pt'><span style='font-size:
     10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman";
     color:#272727'>Install the Intel&reg; RealSense&#8482; RS200 3D Camera. Use 2ea
     M3x18 socket head screws, 2 ea. M3 locknuts, 1 ea. sticky magnet (included
     in camera kit) and 1 ea. cable tie. Use 4ea. (2ea. per screw) M3x1mm
     washers as a spacers between camera and Tower.&nbsp;Refer to </span><a
     href="http://a360.co/2afYBTa"><span class=InternetLink><b><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman"'>Drawing 2C</span></b></span></a><b><u><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#4675A8'>.&nbsp;</span></u></b><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#272727'> The following procedure is recommended
     to install the camera in order to ensure precise horizontal and vertical
     alignment :</span></li>
</ul>

<ol start=1 type=1>
 <li class=MsoNormal style='color:#272727;mso-margin-top-alt:auto;mso-margin-bottom-alt:
     auto;text-align:justify;text-justify:inter-ideograph;line-height:15.75pt;
     mso-list:l3 level1 lfo6;tab-stops:list 36.0pt'><span style='font-size:
     10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman"'>Insert
     1ea. M3x18 screw&nbsp;into slot in camera.<o:p></o:p></span></li>
 <li class=MsoNormal style='color:#272727;mso-margin-top-alt:auto;mso-margin-bottom-alt:
     auto;text-align:justify;text-justify:inter-ideograph;line-height:15.75pt;
     mso-list:l3 level1 lfo6;tab-stops:list 36.0pt'><span style='font-size:
     10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman"'>Add
     2 ea. M3x1mm washers to the screw between camera and Tower.<o:p></o:p></span></li>
 <li class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
     text-align:justify;text-justify:inter-ideograph;line-height:15.75pt;
     mso-list:l3 level1 lfo6;tab-stops:list 36.0pt'><span style='font-size:
     10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman";
     color:#272727'>Loosely secure screw with M3 locknut in the outer camera
     mount hole in the tower.<span style='mso-spacerun:yes'>&nbsp;&nbsp;
     </span>The camera should still be able to rotate and slide freely.</span></li>
 <li class=MsoNormal style='color:#272727;mso-margin-top-alt:auto;mso-margin-bottom-alt:
     auto;text-align:justify;text-justify:inter-ideograph;line-height:15.75pt;
     mso-list:l3 level1 lfo6;tab-stops:list 36.0pt'><span style='font-size:
     10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman"'>Attach
     the magnet to the other side of the camera, making sure it is in an
     accurate position (the bump in the magnet should be in the slot on the
     back of the camera) and remove protective paper from the magnet.<o:p></o:p></span></li>
 <li class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
     text-align:justify;text-justify:inter-ideograph;line-height:15.75pt;
     mso-list:l3 level1 lfo6;tab-stops:list 36.0pt'><span style='font-size:
     10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman";
     color:#272727'>Align camera vertically and horizontally&nbsp;- top edge of
     the camera should be aligned with the tower edge.</span></li>
 <li class=MsoNormal style='color:#272727;mso-margin-top-alt:auto;mso-margin-bottom-alt:
     auto;text-align:justify;text-justify:inter-ideograph;line-height:15.75pt;
     mso-list:l3 level1 lfo6;tab-stops:list 36.0pt'><span style='font-size:
     10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman"'>Attach
     sticky side of the magnet to tower.<o:p></o:p></span></li>
 <li class=MsoNormal style='color:#272727;mso-margin-top-alt:auto;mso-margin-bottom-alt:
     auto;text-align:justify;text-justify:inter-ideograph;line-height:15.75pt;
     mso-list:l3 level1 lfo6;tab-stops:list 36.0pt'><span style='font-size:
     10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman"'>Secure
     the locknut.<o:p></o:p></span></li>
 <li class=MsoNormal style='color:#272727;mso-margin-top-alt:auto;mso-margin-bottom-alt:
     auto;text-align:justify;text-justify:inter-ideograph;line-height:15.75pt;
     mso-list:l3 level1 lfo6;tab-stops:list 36.0pt'><span style='font-size:
     10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman"'>Secure
     the second screw with washers and a locknut<o:p></o:p></span></li>
</ol>

<p class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
margin-left:18.0pt;line-height:15.75pt'><span style='font-size:10.5pt;
font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman";
color:#272727'>Do not overtighten either screw.<span
style='mso-spacerun:yes'>&nbsp;&nbsp; </span>The locknuts and screws should
hold the camera firmly without bending the case.&nbsp;<o:p></o:p></span></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
15.75pt'><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'>2. &nbsp; Top Plate<b>&nbsp;(Drawing
3A, B, C)</b><o:p></o:p></span></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
15.75pt'><b><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</span></b><a
href="http://a360.co/2a3N9qc"><span class=InternetLink><b><span
style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
"Times New Roman";color:#4675A8'>Top 3D Assembly</span></b></span></a></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
15.75pt'><b><u><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#4675A8'><o:p><span
 style='text-decoration:none'>&nbsp;</span></o:p></span></u></b></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
15.75pt'><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'><o:p>&nbsp;</o:p></span></p>

<p class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
margin-left:-18.0pt;line-height:15.75pt;page-break-after:avoid'><span
style='mso-fareast-language:JA;mso-no-proof:yes'><!--[if gte vml 1]><v:shape
 id="Picture_x0020_22" o:spid="_x0000_i1037" type="#_x0000_t75" style='width:423pt;
 height:192pt;visibility:visible;mso-wrap-style:square'>
 <v:imagedata src="INSTRUCTIONS_files/image021.png" o:title=""/>
</v:shape><![endif]--><![if !vml]><img border=0 width=564 height=256
src="INSTRUCTIONS_files/image022.jpg" v:shapes="Picture_x0020_22"><![endif]></span></p>

<p class=MsoCaption>Drawing 3A <span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'><o:p></o:p></span></p>

<ul type=disc>
 <li class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
     line-height:15.75pt;mso-list:l10 level1 lfo7;tab-stops:list 36.0pt'><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#272727'>Attach 2 ea.&nbsp;Inner Mount
     Plates&nbsp;to the Top Plate and secure with 2 ties. Leave ties semi
     loose. Donft cut ties excess length yet. Refer to </span><a
     href="http://a360.co/2a3nbTS"><span class=InternetLink><b><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#4675A8'>Drawing 3A</span></b></span></a><b><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#4675A8'>.</span></b></li>
 <li class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
     line-height:15.75pt;mso-list:l10 level1 lfo7;tab-stops:list 36.0pt'><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#272727'>Install 2 ea. Power On-Off
     switches.&nbsp;Refer to </span><a href="http://a360.co/2a3nbTS"><span
     class=InternetLink><b><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
     mso-fareast-font-family:"Times New Roman";color:#4675A8'>Drawing 3A</span></b></span></a><b><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#4675A8'>.</span></b></li>
</ul>

<p class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
line-height:15.75pt;page-break-after:avoid'><span style='mso-fareast-language:
JA;mso-no-proof:yes'><!--[if gte vml 1]><v:shape id="Picture_x0020_23" o:spid="_x0000_i1036"
 type="#_x0000_t75" style='width:372pt;height:218.25pt;visibility:visible;
 mso-wrap-style:square'>
 <v:imagedata src="INSTRUCTIONS_files/image023.png" o:title=""/>
</v:shape><![endif]--><![if !vml]><img border=0 width=496 height=291
src="INSTRUCTIONS_files/image024.jpg" v:shapes="Picture_x0020_23"><![endif]></span></p>

<p class=MsoCaption>Drawing 3B <span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'><o:p></o:p></span></p>

<ul type=disc>
 <li class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
     text-align:justify;text-justify:inter-ideograph;line-height:15.75pt;
     mso-list:l10 level1 lfo7;tab-stops:list 36.0pt'><span style='font-size:
     10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman";
     color:#272727'>Attach Top Plate to the Tower with 2 ea. M3x6 screws,
     M3nuts, M3x1 washers in T-slots. Tighten ties and cut off excess of tiesf
     length. Refer to </span><a href="http://a360.co/2a3n1Mc"><span
     class=InternetLink><b><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
     mso-fareast-font-family:"Times New Roman";color:#4675A8'>Drawing 3B</span></b></span></a><b><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#4675A8'>.</span></b></li>
 <li class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
     text-align:justify;text-justify:inter-ideograph;line-height:15.75pt;
     mso-list:l10 level1 lfo7;tab-stops:list 36.0pt'><span style='font-size:
     10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman";
     color:#272727'>Install 2 ea. M3x51mm standoffs. Use 2ea. M3x8 button head
     screws.&nbsp;Refer to </span><a href="http://a360.co/2a3n1Mc"><span
     class=InternetLink><b><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
     mso-fareast-font-family:"Times New Roman";color:#4675A8'>Drawing 3B</span></b></span></a><b><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#272727'>.</span></b></li>
 <li class=MsoNormal style='color:#272727;mso-margin-top-alt:auto;mso-margin-bottom-alt:
     auto;text-align:justify;text-justify:inter-ideograph;line-height:15.75pt;
     mso-list:l10 level1 lfo7;tab-stops:list 36.0pt'><span style='font-size:
     10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman"'>Secure
     Outer Mount Plate to the Servo with 8ea. M2x8&nbsp;screws and 8ea. M2 hex
     nuts<b>.&nbsp;</b>Repeat for the second Outer Mount. <o:p></o:p></span></li>
</ul>

<p class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
line-height:15.75pt;page-break-after:avoid'><span style='mso-fareast-language:
JA;mso-no-proof:yes'><!--[if gte vml 1]><v:shape id="Picture_x0020_31" o:spid="_x0000_i1035"
 type="#_x0000_t75" style='width:404.25pt;height:235.5pt;visibility:visible;
 mso-wrap-style:square'>
 <v:imagedata src="INSTRUCTIONS_files/image025.png" o:title=""/>
</v:shape><![endif]--><![if !vml]><img border=0 width=539 height=314
src="INSTRUCTIONS_files/image026.jpg" v:shapes="Picture_x0020_31"><![endif]></span></p>

<p class=MsoCaption>Drawing 3C <span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'><o:p></o:p></span></p>

<ul type=disc>
 <li class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
     line-height:15.75pt;mso-list:l10 level1 lfo7;tab-stops:list 36.0pt'><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#272727'>Attach Outer Mount Plates to Top Plate
     with ties. Refer to </span><a href="http://a360.co/2a3orGD"><span
     class=InternetLink><b><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
     mso-fareast-font-family:"Times New Roman";color:#4675A8'>Drawing 3C</span></b></span></a><b><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#4675A8'>.<span style='mso-spacerun:yes'>&nbsp;
     </span></span></b></li>
</ul>

<p class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
line-height:15.75pt;page-break-after:avoid'><span style='mso-fareast-language:
JA;mso-no-proof:yes'><!--[if gte vml 1]><v:shape id="Picture_x0020_32" o:spid="_x0000_i1034"
 type="#_x0000_t75" style='width:192pt;height:184.5pt;visibility:visible;
 mso-wrap-style:square'>
 <v:imagedata src="INSTRUCTIONS_files/image027.png" o:title=""/>
</v:shape><![endif]--><![if !vml]><img border=0 width=256 height=246
src="INSTRUCTIONS_files/image028.jpg" v:shapes="Picture_x0020_32"><![endif]><!--[if gte vml 1]><v:shape
 id="Picture_x0020_33" o:spid="_x0000_i1033" type="#_x0000_t75" style='width:228.75pt;
 height:233.25pt;visibility:visible;mso-wrap-style:square'>
 <v:imagedata src="INSTRUCTIONS_files/image029.png" o:title=""/>
</v:shape><![endif]--><![if !vml]><img border=0 width=305 height=311
src="INSTRUCTIONS_files/image030.jpg" v:shapes="Picture_x0020_33"><![endif]></span></p>

<p class=MsoCaption>Drawing 4 <span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'><o:p></o:p></span></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;text-align:
justify;text-justify:inter-ideograph;line-height:15.75pt'><span
style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
"Times New Roman";color:#272727'>&nbsp;&nbsp; 3.&nbsp; Install axles and
Wheels, (</span><a href="http://a360.co/2a3nPRi"><span class=InternetLink><b><span
style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
"Times New Roman";color:#4675A8'>Drawing 4</span></b></span></a><b><span
style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
"Times New Roman";color:#272727'>,<span class=GramE>&nbsp;&nbsp;</span></span></b><a
href="http://a360.co/2a3N1Y2"><span class=InternetLink><b><span
style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
"Times New Roman";color:#4675A8'>3D Assembly</span></b></span></a><b><span
style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
"Times New Roman";color:#272727'>)</span></b><span style='font-size:10.5pt;
font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman";
color:#272727'>&nbsp;but do not secure the M5 axle nut tightly yet. Repeat for
the second Wheel.<span style='mso-spacerun:yes'>&nbsp;&nbsp; </span>NOTE: it is
possible to substitute an M6 nut and bolt for the shoulder bolt used here, as
long as at least 12mm of the shaft of the bolt is unthreaded and is actually
6mm in diameter.</span></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;text-align:
justify;text-justify:inter-ideograph;line-height:15.75pt'><b><span
style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
"Times New Roman";color:#272727'>At this point, for the ease of future
assembly, it is advisable to address Servo wiring&nbsp;as described in Servo
Wiring section below.</span></b><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'><o:p></o:p></span></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
15.75pt'><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'>&nbsp;<o:p></o:p></span></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
15.75pt'><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'>4. Base Plate<b>&nbsp;(Drawings
5A, B)</b><o:p></o:p></span></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
15.75pt'><b><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</span></b><a
href="http://a360.co/2agoFxm"><span class=InternetLink><b><span
style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
"Times New Roman";color:#4675A8'>Base 3D Assembly</span></b></span></a></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
15.75pt'><b><u><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#4675A8'><o:p><span
 style='text-decoration:none'>&nbsp;</span></o:p></span></u></b></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
15.75pt;page-break-after:avoid'><span style='mso-fareast-language:JA;
mso-no-proof:yes'><!--[if gte vml 1]><v:shape id="Picture_x0020_34" o:spid="_x0000_i1032"
 type="#_x0000_t75" style='width:363.75pt;height:179.25pt;visibility:visible;
 mso-wrap-style:square'>
 <v:imagedata src="INSTRUCTIONS_files/image031.png" o:title=""/>
</v:shape><![endif]--><![if !vml]><img border=0 width=485 height=239
src="INSTRUCTIONS_files/image032.jpg" v:shapes="Picture_x0020_34"><![endif]></span></p>

<p class=MsoCaption>Drawing 5A <span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'><o:p></o:p></span></p>

<ul type=disc>
 <li class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
     text-align:justify;text-justify:inter-ideograph;line-height:15.75pt;
     mso-list:l6 level1 lfo8;tab-stops:list 36.0pt'><span style='font-size:
     10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman";
     color:#272727'>Install the ball caster on the Base plate with 3ea.
     M3x8&nbsp;screws and 3ea. M3 locknuts. . Current design uses only one
     caster in the rear part of the robot, but you may add 2 more front casters
     if you want; this will reduce the ability of the robot to go over
     obstacles but will also make it less liable to tipping forward when
     decelerating. The extra mounting holes on the front of the Base plate
     provide this option.&nbsp;Refer to </span><a href="http://a360.co/2a3zOys"><span
     class=InternetLink><b><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
     mso-fareast-font-family:"Times New Roman";color:#4675A8'>Drawing 5A</span></b></span></a><b><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#4675A8'>. </span></b><b><span style='font-size:
     10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman";
     color:black'><span style='mso-spacerun:yes'>&nbsp;</span>Note also that
     only the <span class=SpellE>Acetal</span> version of the base plate have
     flexible gsuspensionsh for the casters.<span
     style='mso-spacerun:yes'>&nbsp; </span>If cut in Acrylic these have proven
     to break too easily.<span style='mso-spacerun:yes'>&nbsp;&nbsp; </span></span></b><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:black;mso-bidi-font-weight:bold'>The frame is
     intentionally designed so that the casters do not all touch the ground at
     the same time in any case.</span></li>
</ul>

<p class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
line-height:15.75pt;page-break-after:avoid'><span style='font-size:10.5pt;
font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman";
color:#272727'><span
style='mso-spacerun:yes'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
</span></span><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727;mso-fareast-language:
JA;mso-no-proof:yes'><!--[if gte vml 1]><v:shape id="Picture_x0020_35" o:spid="_x0000_i1031"
 type="#_x0000_t75" style='width:469.5pt;height:315pt;visibility:visible;
 mso-wrap-style:square'>
 <v:imagedata src="INSTRUCTIONS_files/image033.png" o:title=""/>
</v:shape><![endif]--><![if !vml]><img border=0 width=626 height=420
src="INSTRUCTIONS_files/image034.jpg" v:shapes="Picture_x0020_35"><![endif]></span></p>

<p class=MsoCaption>Drawing 7 <span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'><o:p></o:p></span></p>

<ul type=disc>
 <li class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
     line-height:15.75pt;mso-list:l6 level1 lfo8;tab-stops:list 36.0pt'><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#272727'>&nbsp;Insert Velcro straps in the slots
     of the Base Plate and Install the Battery. Refer to </span><a
     href="http://a360.co/2a88AGJ"><span class=InternetLink><b><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#4675A8'>Drawing 7</span></b></span></a><b><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#4675A8'>.<span style='mso-spacerun:yes'>&nbsp;
     </span></span></b></li>
 <li class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
     line-height:15.75pt;mso-list:l6 level1 lfo8;tab-stops:list 36.0pt'><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";mso-bidi-font-weight:bold'>You should remove the battery
     after fitting it, then add it back in afterwards.</span><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman"'><o:p></o:p></span></li>
</ul>

<p class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
line-height:15.75pt;page-break-after:avoid'><span style='font-size:10.5pt;
font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman";
color:#272727'><span
style='mso-spacerun:yes'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
</span></span><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727;mso-fareast-language:
JA;mso-no-proof:yes'><!--[if gte vml 1]><v:shape id="Picture_x0020_36" o:spid="_x0000_i1030"
 type="#_x0000_t75" style='width:340.5pt;height:229.5pt;visibility:visible;
 mso-wrap-style:square'>
 <v:imagedata src="INSTRUCTIONS_files/image035.png" o:title=""/>
</v:shape><![endif]--><![if !vml]><img border=0 width=454 height=306
src="INSTRUCTIONS_files/image036.gif" v:shapes="Picture_x0020_36"><![endif]></span></p>

<p class=MsoCaption>Drawing 5B <span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'><o:p></o:p></span></p>

<ul type=disc>
 <li class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
     line-height:15.75pt;mso-list:l6 level1 lfo8;tab-stops:list 36.0pt'><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#272727'>Secure front Standoffs to the Base Plate
     with 2 ea. M3x8 screws&nbsp;</span><a href="http://a360.co/2a3zYpA"><span
     class=InternetLink><b><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
     mso-fareast-font-family:"Times New Roman";color:#4675A8'>Drawing 5B</span></b></span></a></li>
 <li class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
     line-height:15.75pt;mso-list:l6 level1 lfo8;tab-stops:list 36.0pt'><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#272727'>Attach Base Plate to the Tower and secure
     it with 2ea. M3x6screws, 3ea. M3 nuts, and 3ea. M3x1mm washers in T-slots.
     Refer to </span><a href="http://a360.co/2a3zYpA"><span class=InternetLink><b><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#4675A8'>Drawing 5B</span></b></span></a></li>
 <li class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
     line-height:15.75pt;mso-list:l6 level1 lfo8;tab-stops:list 36.0pt'><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#272727'>Secure Outer and Inner Mount Plates to
     the Base Plate with ties. (On both sides).&nbsp;Refer to </span><a
     href="http://a360.co/2a3zYpA"><span class=InternetLink><b><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#4675A8'>Drawing 5B</span></b></span></a></li>
 <li class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
     line-height:15.75pt;mso-list:l6 level1 lfo8;tab-stops:list 36.0pt'><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#272727'>Tighten Axel bolt with a wrench. Do not
     overtighten. Check that Wheels rotate freely on the Axel.&nbsp;Refer to </span><a
     href="http://a360.co/2a3zYpA"><span class=InternetLink><b><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#4675A8'>Drawing 5B</span></b></span></a><u><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#4675A8'>.</span></u></li>
</ul>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
15.75pt'><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'>&nbsp;&nbsp; 5.
Install driver and O-rings. (<b>Drawings 6 A, B)</b><o:p></o:p></span></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
15.75pt'><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'>&nbsp;&nbsp;&nbsp;&nbsp;</span><a
href="http://a360.co/2a82vKJ"><span class=InternetLink><span style='font-size:
10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman";
color:#4675A8'>&nbsp;</span></span></a><a href="http://a360.co/2akV7Pf"><span
class=InternetLink><b><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#4675A8'>Driver and O-Ring
installation 3D</span></b></span></a></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
15.75pt'><b><u><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#4675A8'><o:p><span
 style='text-decoration:none'>&nbsp;</span></o:p></span></u></b></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
15.75pt;page-break-after:avoid'><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'><span
style='mso-spacerun:yes'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
</span></span><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727;mso-fareast-language:
JA;mso-no-proof:yes'><!--[if gte vml 1]><v:shape id="Picture_x0020_37" o:spid="_x0000_i1029"
 type="#_x0000_t75" style='width:227.25pt;height:163.5pt;visibility:visible;
 mso-wrap-style:square'>
 <v:imagedata src="INSTRUCTIONS_files/image037.png" o:title=""/>
</v:shape><![endif]--><![if !vml]><img border=0 width=303 height=218
src="INSTRUCTIONS_files/image038.jpg" v:shapes="Picture_x0020_37"><![endif]></span></p>

<p class=MsoCaption>Drawing 6A <span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'><o:p></o:p></span></p>

<ul type=disc>
 <li class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
     line-height:15.75pt;mso-list:l0 level1 lfo9;tab-stops:list 36.0pt'><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#272727'>Assemble driver from Driver components in
     your hand. Parts will be loose.&nbsp;</span><a
     href="http://a360.co/2a813Yx"><span class=InternetLink><b><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#4675A8'>Drawing 6A</span></b></span></a><b><u><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#4675A8'><span style='mso-spacerun:yes'>&nbsp;
     </span></span></u></b>Using<span style='mso-spacerun:yes'>&nbsp;&nbsp;
     </span>4 ea. M2x20mm screws secure driver on the servo hub.</li>
</ul>

<p class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
margin-left:36.0pt;line-height:15.75pt'><span style='font-size:10.5pt;
font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman";
color:#272727'><o:p>&nbsp;</o:p></span></p>

<p class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
line-height:15.75pt;page-break-after:avoid'><span style='mso-fareast-language:
JA;mso-no-proof:yes'><!--[if gte vml 1]><v:shape id="Picture_x0020_38" o:spid="_x0000_i1028"
 type="#_x0000_t75" style='width:398.25pt;height:276.75pt;visibility:visible;
 mso-wrap-style:square'>
 <v:imagedata src="INSTRUCTIONS_files/image039.png" o:title=""/>
</v:shape><![endif]--><![if !vml]><img border=0 width=531 height=369
src="INSTRUCTIONS_files/image040.jpg" v:shapes="Picture_x0020_38"><![endif]></span></p>

<p class=MsoCaption>Drawing 6B <span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'><o:p></o:p></span></p>

<p class=MsoListParagraph style='mso-margin-top-alt:auto;mso-margin-bottom-alt:
auto;mso-add-space:auto;text-indent:-18.0pt;line-height:15.75pt;mso-list:l0 level1 lfo9;
tab-stops:list 36.0pt'><![if !supportLists]><span style='font-size:10.5pt;
mso-bidi-font-size:11.0pt;font-family:Symbol;mso-fareast-font-family:Symbol;
mso-bidi-font-family:Symbol'><span style='mso-list:Ignore'>&middot;<span
style='font:7.0pt "Times New Roman"'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
</span></span></span><![endif]><b><span style='font-size:10.5pt;font-family:
"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman";color:#272727'>I</span></b><span
style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
"Times New Roman";color:#272727'>nstall Driver and O-Ring.&nbsp;</span><a
href="http://a360.co/2akV0mX"><span class=InternetLink><b><span
style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
"Times New Roman";color:#4675A8'>Drawing 6B</span></b></span></a><span
style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
"Times New Roman";color:#272727;mso-bidi-font-weight:bold'> by placing O-Ring
over the Driver and then snapping it into the grooves on the Wheel.</span></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
15.75pt'><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'>6.&nbsp;&nbsp;</span><a
href="http://a360.co/2akZt8Y"><span class=InternetLink><b><span
style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
"Times New Roman";color:#4675A8'>3DAssembly&nbsp;with battery installed</span></b></span></a></p>

<b><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
"Times New Roman";color:#4675A8;mso-ansi-language:EN-US;mso-fareast-language:
ZH-CN;mso-bidi-language:AR-SA'><br clear=all style='mso-special-character:line-break;
page-break-before:always'>
</span></b>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
normal'><b><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#4675A8'><o:p>&nbsp;</o:p></span></b></p>

<h1>Wiring Components</h1>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
normal;mso-outline-level:2'><span style='font-size:18.0pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:blue'><o:p>&nbsp;</o:p></span></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
normal;mso-outline-level:2'><span style='mso-fareast-language:JA;mso-no-proof:
yes'><!--[if gte vml 1]><v:shape id="Picture_x0020_24" o:spid="_x0000_i1027"
 type="#_x0000_t75" style='width:303pt;height:234.75pt;visibility:visible;
 mso-wrap-style:square'>
 <v:imagedata src="INSTRUCTIONS_files/image041.png" o:title=""/>
</v:shape><![endif]--><![if !vml]><img border=0 width=404 height=313
src="INSTRUCTIONS_files/image042.jpg" v:shapes="Picture_x0020_24"><![endif]></span><span
style='font-size:18.0pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
"Times New Roman";color:#272727'><o:p></o:p></span></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;text-align:
justify;text-justify:inter-ideograph;line-height:15.75pt'><span
style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
"Times New Roman";color:#272727'>1. Electrical power supply.&nbsp;</span><a
href="http://a360.co/2a8p4yC"><span class=InternetLink><b><span
style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
"Times New Roman";color:#4675A8'>Power Supply diagram</span></b></span></a></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;text-align:
justify;text-justify:inter-ideograph;line-height:15.75pt'><span
style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
"Times New Roman";color:#272727'>In this design we are using&nbsp;Venom 20C 3S
4000mAh 11.1V <span class=SpellE>LiPo</span> Battery with Universal Plug. In
our tests this battery provides enough power for 60 min of robot's
operation.&nbsp;<o:p></o:p></span></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;text-align:
justify;text-justify:inter-ideograph;line-height:15.75pt'><span
style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
"Times New Roman";color:#272727'>Main electrical power is distributed from the
battery through an in-line 5 AMP fuse.<span
style='mso-spacerun:yes'>&nbsp;&nbsp; </span>The positive side is routed
through the main power rocker switch to the DC-DC convertor, where it splits
into 5V and 12V supply lines. The 12V line&nbsp;is used to power the Servos using
a modified <span class=SpellE>Dynamixel</span> cable while the 5V line is used
to power UP board with a barrel connector.<o:p></o:p></span></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;text-align:
justify;text-justify:inter-ideograph;line-height:15.75pt'><span
style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
"Times New Roman";color:#272727'>Pay special attention to the correct polarity
of the wires when connecting to DC-DC convertor.<span
style='mso-spacerun:yes'>&nbsp; </span>If you get this wrong you can destroy
your UP Board and/or your servos.<span style='mso-spacerun:yes'>&nbsp;&nbsp;
</span>It is <b>strongly</b> recommended to use a voltmeter to check both the
output voltage and the polarity before plugging in the servos or the UP
Board!&nbsp;(<span class=GramE>see</span> </span><a
href="http://www.dfrobot.com/index.php?route=product/product&amp;product_id=752#.V5etZTU2eXg"><span
class=InternetLink><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#4675A8'>Manufacturer
instructions</span></span></a><u><span style='font-size:10.5pt;font-family:
"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman";color:#4675A8'>).</span></u></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
15.75pt'><b><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'>Also be careful when
wiring up the power from the battery.<span style='mso-spacerun:yes'>&nbsp;
</span>We know of at least one person that has completely destroyed a robot by
wiring up their battery backwards.</span></b></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
15.75pt'><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'><o:p>&nbsp;</o:p></span></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
15.75pt'><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'>2. Servo Wiring (For
the ease of assembly it is advisable to complete this step after step 3 of the
mechanical assembly)<o:p></o:p></span></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
15.75pt'><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'><o:p>&nbsp;</o:p></span></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
15.75pt'><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'><o:p>&nbsp;</o:p></span></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
15.75pt'><span style='mso-fareast-language:EN-US'><span
style='mso-spacerun:yes'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
</span></span><span style='mso-fareast-language:JA;mso-no-proof:yes'><!--[if gte vml 1]><v:shape
 id="Picture_x0020_25" o:spid="_x0000_i1026" type="#_x0000_t75" style='width:321.75pt;
 height:211.5pt;visibility:visible;mso-wrap-style:square'>
 <v:imagedata src="INSTRUCTIONS_files/image043.png" o:title=""/>
</v:shape><![endif]--><![if !vml]><img border=0 width=429 height=282
src="INSTRUCTIONS_files/image044.jpg" v:shapes="Picture_x0020_25"><![endif]></span><span
style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
"Times New Roman";color:#272727'><o:p></o:p></span></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
15.75pt'><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'><o:p>&nbsp;</o:p></span></p>

<ul type=disc>
 <li class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
     text-align:justify;text-justify:inter-ideograph;line-height:15.75pt;
     mso-list:l8 level1 lfo10;tab-stops:list 36.0pt'><a
     href="http://a360.co/2a8p5CN"><span class=InternetLink><b><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#4675A8'>Servo wiring Diagram</span></b></span></a></li>
 <li class=MsoNormal style='color:#272727;mso-margin-top-alt:auto;mso-margin-bottom-alt:
     auto;text-align:justify;text-justify:inter-ideograph;line-height:15.75pt;
     mso-list:l8 level1 lfo10;tab-stops:list 36.0pt'><span style='font-size:
     10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman"'>Connect
     USB2AX to 1st servo using a 250mm <span class=SpellE>Dynamixel</span>
     cable.<o:p></o:p></span></li>
 <li class=MsoNormal style='color:#272727;mso-margin-top-alt:auto;mso-margin-bottom-alt:
     auto;text-align:justify;text-justify:inter-ideograph;line-height:15.75pt;
     mso-list:l8 level1 lfo10;tab-stops:list 36.0pt'><span style='font-size:
     10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman"'>Interconnect
     both servos with <span class=SpellE>Dynamixel</span> 100mm cable<o:p></o:p></span></li>
 <li class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
     text-align:justify;text-justify:inter-ideograph;line-height:15.75pt;
     mso-list:l8 level1 lfo10;tab-stops:list 36.0pt'><span style='font-size:
     10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman";
     color:#272727'>Using gcustomh cable, connect the 2nd servo to pass-through
     power connector of DC-DC power convertor.<span
     style='mso-spacerun:yes'>&nbsp; </span>This cable can be created by
     modifying a normal <span class=SpellE>Dynamixel</span> cable.<span
     style='mso-spacerun:yes'>&nbsp; </span>Pay attention to the polarity of
     wires.&nbsp;<u>The gredh wire (the center wire of the <span class=SpellE>Dynamixel</span>
     cable) connects to OV out on the DC-DC convertor and the gblackh wire
     connects to GND</u>. See wiring diagram, and note especially the keying on
     the <span class=SpellE>Dynamixel</span> cables.<span
     style='mso-spacerun:yes'>&nbsp; </span>Unfortunately you will have to use
     the connector keying to distinguish the ground and the signal cables since
     they are not actually colored or labelled in any way.</span></li>
 <li class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
     text-align:justify;text-justify:inter-ideograph;line-height:15.75pt;
     mso-list:l8 level1 lfo10;tab-stops:list 36.0pt'><span style='font-size:
     10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman";
     color:#272727'>The gredh wire of the custom cable has 2 female connectors
     that should be attached to the second rocker switch. This rocker switch
     will control power supply to motors only, which will allow you to turn off
     motors, but keep the UP board powered. This is useful both to preserve
     battery life when you need to work on UP board configuration and
     programming, and to stop or reset the motors if necessary without crashing
     the UP Board.</span></li>
</ul>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
15.75pt'><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'>3. Camera and UP
Board.<o:p></o:p></span></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
15.75pt'><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'><o:p>&nbsp;</o:p></span></p>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
15.75pt'><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'><span
style='mso-spacerun:yes'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
</span></span><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727;mso-fareast-language:
JA;mso-no-proof:yes'><!--[if gte vml 1]><v:shape id="Picture_x0020_28" o:spid="_x0000_i1025"
 type="#_x0000_t75" style='width:330.75pt;height:218.25pt;visibility:visible;
 mso-wrap-style:square'>
 <v:imagedata src="INSTRUCTIONS_files/image045.png" o:title=""/>
</v:shape><![endif]--><![if !vml]><img border=0 width=441 height=291
src="INSTRUCTIONS_files/image046.jpg" v:shapes="Picture_x0020_28"><![endif]></span><span
style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
"Times New Roman";color:#272727'><span
style='mso-spacerun:yes'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</span><o:p></o:p></span></p>

<ul type=disc>
 <li class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
     text-align:justify;text-justify:inter-ideograph;line-height:15.75pt;
     mso-list:l4 level1 lfo11;tab-stops:list 36.0pt'><a
     href="http://a360.co/2ci6REp"><span class=InternetLink><span
     style='font-size:10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:
     "Times New Roman";color:#4675A8'>Connection Diagram</span></span></a></li>
 <li class=MsoNormal style='color:#272727;mso-margin-top-alt:auto;mso-margin-bottom-alt:
     auto;text-align:justify;text-justify:inter-ideograph;line-height:15.75pt;
     mso-list:l4 level1 lfo11;tab-stops:list 36.0pt'><span style='font-size:
     10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman"'>Connect
     RealSense R200 Camera to USB3 OTG port with the USB 3.0 cable provided in
     the camera kit.<o:p></o:p></span></li>
 <li class=MsoNormal style='color:#272727;mso-margin-top-alt:auto;mso-margin-bottom-alt:
     auto;text-align:justify;text-justify:inter-ideograph;line-height:15.75pt;
     mso-list:l4 level1 lfo11;tab-stops:list 36.0pt'><span style='font-size:
     10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman"'>Plug
     in USB2AX adapter&nbsp;into USB2 port of the UP Board.<o:p></o:p></span></li>
 <li class=MsoNormal style='mso-margin-top-alt:auto;mso-margin-bottom-alt:auto;
     text-align:justify;text-justify:inter-ideograph;line-height:15.75pt;
     mso-list:l4 level1 lfo11;tab-stops:list 36.0pt'><span style='font-size:
     10.5pt;font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman";
     color:#272727'>It is advisable to route camera's&nbsp;and USB2AX cables
     between Tower and UP Board. Address cable management and secure UP Board to
     the Tower plate with 4ea. M2.5x6 screws.</span> <a
     href="http://a360.co/2ajdTqA"><span class=InternetLink>Drawing 8</span></a></li>
 <li class=MsoNormal style='color:#272727;mso-margin-top-alt:auto;mso-margin-bottom-alt:
     auto;text-align:justify;text-justify:inter-ideograph;line-height:15.75pt;
     mso-list:l4 level1 lfo11;tab-stops:list 36.0pt'><span style='color:windowtext'>Insert
     Wi-Fi USB dongle in UP board</span><span style='font-size:10.5pt;
     font-family:"Helvetica",sans-serif;mso-fareast-font-family:"Times New Roman"'><o:p></o:p></span></li>
</ul>

<p class=MsoNormal style='margin-bottom:0cm;margin-bottom:.0001pt;line-height:
15.75pt'><span style='font-size:10.5pt;font-family:"Helvetica",sans-serif;
mso-fareast-font-family:"Times New Roman";color:#272727'>This section completes
Mechanical and Wiring assembly of SAWR.<o:p></o:p></span></p>

<p class=MsoNormal><o:p>&nbsp;</o:p></p>

</div>

</body>

</html>
