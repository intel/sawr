#!/bin/sh
rosrun map_server map_saver map:=map -f $(rospack find sawr_mapping)/maps/saved

