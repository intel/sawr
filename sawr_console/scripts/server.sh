#!/bin/sh
# arg 1: directory to run Node.js app in
# arg 2: port to use (recommended: 8080)
echo "Starting Node.js Express server on port $2 for SAWR web console"
cd $1/app
nodejs server.js $2
