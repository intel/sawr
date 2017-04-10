Simple Autonomous Wheeled Robot (SAWR) Hardware
===============================================

![OpenSCAD Rendering of SAWR Model](Images/iso.png)

Here is all the information you need to build your own SAWR!

  1. Bills of materials (BOMs) are located under [Materials](Materials). Visit
     the [SAWR project page at 01.org](https://01.org/sawr) to see if kits are
     available. Alternatively you can just source all the parts yourself; links
     are provided to McMaster-Carr and Amazon but other sources can also be
     used as the parts are generally standard.

  2. If you want to cut your own frame, laser-cut files are located in the
     [Make](Make) subdirectory. Choose one of the following depending on your
     material:

     * [Acrylic 3mm sheet](Make/3mm_acrylic.pdf)
     * [POM 3mm sheet](Make/3mm_pom.pdf)

     POM (short for Polyoxymethylene, also known as Acetal, Delrin, or Duracon)
     is highly recommended if you can source it.  Acyrlic (short for polymethyl
     methacrylate, also known as PMMA, Plexiglas, Acrylite, or Lucite) is ok
     but tends to be brittle. If your sheets have different dimensions than
     what the cut sheets are laid out for (350mm x 400mm) you may have to
     reorganize the parts using a vector graphics editor such as
     [Inkscape](https://inkscape.org/). In this case you may want to start with
     the SVG files provided rather than the PDF, although exporting to PDF is
     recommended to better preserve units. Individual parts and DXF files are
     in the [Parts](Parts) subdirectory if you need them. There are also some
     variants of some parts there, for instance without accessory mounting
     holes.
  
     You should also cut one of the following:

     * [2mm sheet (same for all materials)](Make/2mm.pdf)

     Note that both 2mm and 3mm parts are used and these are organized onto
     different sheets. The design allows for 10% tolerance in sheet thickness,
     but is really only important for the 3mm sheet, due to the use of slots
     and tabs. Consider this if you can only source sheets in Imperial
     thicknesses. In particular, 1/8 inch sheet is 3.175mm and is within 10%
     of 3mm. Unfortunately 2mm sheet is a little harder to find a close
     replacement for in Imperial units. A 3/32 inch sheet is 2.38mm and
     although slightly outside the 10% tolerance is a reasonable replacement
     (you might have to lengthen the bolts in the wheels slightly, however...
     this is left as an exercise for the reader).

  3. Under the [Models](Models)  subdirectory you will find the source CAD
     files. SAWR is modeled using [OpenSCAD](http://www.openscad.org/), an
     open-source 3D parametric modeling system. Using OpenSCAD, models are
     actually described using a textual language, not a direct manipulation
     graphical interface. The main file is `frame.scad`.

  4. Build instructions are located in [ASSEMBLY.md](ASSEMBLY.md).

  5. Further information on the build process, videos, etc will be posted to
     the [SAWR project at 01.org](https://01.org/sawr).
