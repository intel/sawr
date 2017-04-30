#/bin/sh
# Start a terminal window with a font that scale well under Ubuntu and 
# with the -hold option so it stays up when script exits (so you can see if
# something went wrong)
xterm -sb -sl 1024 -b 10 -bd green -j -s -fa Monospace -fs 12 -hold -e $1

