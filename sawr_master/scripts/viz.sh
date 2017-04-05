#/bin/sh
# Start some visualizers for debugging/demos
#echo "Starting rqt_graph..."
#rqt_graph &
#echo "Starting rqt_plot..."
#rqt_plot &
echo "Starting rviz..."
roslaunch sawr_navigation display.launch &
