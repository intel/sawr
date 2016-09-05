#!/bin/sh
rosrun map_server map_saver map:=map -f $(rospack find whr_mapping)/maps/saved

