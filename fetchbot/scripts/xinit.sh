#/bin/sh
echo "Initiating Phase $1 in a new xterm"
xterm -e ./scripts/init_$1.sh &

