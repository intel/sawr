#/bin/sh
# Start a SAWR Phase in a terminal (with input disabled)
echo "Initiating SAWR Phase $1 in a new xterm"
BASE=`rospack find sawr_master`/scripts
$BASE/xm.sh "$BASE/init_$1.sh" &
