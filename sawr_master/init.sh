#/bin/sh
# roscore not needed, roslaunch will start if necessary
BASE=`rospack find sawr_master`/scripts
$BASE/xm.sh "roslaunch sawr_master init.launch"

