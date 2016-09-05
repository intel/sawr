FetchBot Master Package

This package serves two purposes:
1. To act as a metapackage in that is has 
   dependencies on all other packages needed for the FetchBot.
   Therefore, you only have to install this package to get
   everything.
2. Includes launch files for startup.
   These are organized in "phases" to resolve ordering dependencies.   
   Launch init_1, then init_2, etc.  It is useful to launch each of these
   in a separate window.  Wait for each to stabilize before starting the 
   next.  




