#!/bin/sh
# Look up the current governor of each core
# Enable "ondemand" governor for all CPU cores
for CPUFREQ in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do [ -f $CPUFREQ ] || continue; cat $CPUFREQ; done

