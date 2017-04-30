#/bin/sh
# Start a particular phase as a background process (quiet launch)
echo "Initiating Phase $1 as a background process"
BASE=`rospack find sawr_master`/scripts
$BASE/init_$1.sh &

