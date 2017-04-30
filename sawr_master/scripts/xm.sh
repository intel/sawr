#/bin/sh
# Like xt, but disables write access (so only for outputting messages)
xterm -sb -sl 1024 -b 10 -bd red -j -s -mesg -fa Monospace -fs 12 -hold -e $1

