#/bin/sh
# Start alt SAWR Phase in a terminal
echo "Initiating SAWR Phase $1 (alt) in a new xterm"
BASE=`rospack find sawr_master`/scripts
$BASE/xm.sh "./scripts/init_alt_$1.sh" &

