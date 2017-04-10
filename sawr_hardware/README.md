Simple Autonomous Wheeled Robot (SAWR) Hardware
===============================================

![OpenSCAD Rendering of SAWR Model](Images/iso.png)

Here is all the information you need to build your own SAWR!

  1. Bills of materials are located under [Materials](Materials). Visit the
     [SAWR project page at 01.org](https://01.org/sawr) to see if kits are
     available. Alternatively you can just source all the parts yourself; links
     are provided to McMaster-Carr and Amazon but other sources can also be 
     used as the parts are generally standard.

  2. If you want to cut your own frame, laser-cut files are located in the
     [Make](Make) subdirectory. Use the version appropriate to your material. 
     POM (short for Polyoxymethylene, also known as Acetal, Delrin, or Duracon)
     is highly recommended if you can source it.  Acyrlic (short for polymethyl
     methacrylate, also known as PMMA, Plexiglas, Acrylite, or Lucite) is ok
     but tends to be brittle. If your sheets have different dimensions than
     the cut files you may have to reorganize the parts a little bit using a
     vector graphics editor such as [Inkscape](https://inkscape.org/).

     Note that both 2mm and 3mm parts are used and these are organized onto 
     different sheets. The design allows for 10% tolerance in sheet thickness, 
     and is really only important for the 3mm sheet. Consider this if you can 
     only source sheets in Imperial thicknesses. In particular, 1/8 inch sheet
     is 3.175mm and is within 10% of 3mm.  Unfortunately 2mm sheet is a little
     harder to find a close replacement for. A 3/32 inch sheet is 2.38mm and 
     although outside the 10% tolerance is a reasonable replacement (you might 
     have to lengthen the bolts in the wheels slightly, however... this is left
     as an exercise for the reader).  

  3. Under the Models subdirectory you will find the CAD files. SAWR is 
     modeled using [OpenSCAD](http://www.openscad.org/), an open-source 3D 
     parametric modeling system. The main file is ``frame.scad``.

  4. Build instructions are located in 
     [Assembly/INSTRUCTIONS.md](Assembly/INSTRUCTIONS.md).

  5. Further information on the build process, videos, etc will be posted to
     the [SAWR project at 01.org](https://01.org/sawr).
