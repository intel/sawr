#!/bin/sh
# Enable "performance" governor for all CPU cores
# This is especially useful on an Intel Joule with active cooling
# NOTE: requires "sudo" to execute
for CPUFREQ in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do [ -f $CPUFREQ ] || continue; echo -n performance > $CPUFREQ; done
