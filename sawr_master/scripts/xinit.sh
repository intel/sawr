#/bin/sh
# Start a SAWR Phase in a terminal
echo "Initiating SAWR Phase $1 in a new xterm"
xterm -e ./scripts/init_$1.sh &

