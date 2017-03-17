#!/bin/sh
# Enable "ondemand" governor for all CPU cores
# NOTE: requires "sudo" to execute
for CPUFREQ in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do [ -f $CPUFREQ ] || continue; echo -n ondemand > $CPUFREQ; done
