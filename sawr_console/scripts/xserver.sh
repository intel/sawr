#!/bin/sh
echo "Bringing up the SAWR server..."
xterm -e ./server.sh $1 $2 &
