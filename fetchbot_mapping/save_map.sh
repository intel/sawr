#!/bin/sh
rosrun map_server map_saver map:=map -f $(rospack find fetchbot_mapping)/maps/saved

