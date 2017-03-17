#!/bin/sh
# Look up the current clock rate of each core
# Preface with "watch" if you want to see it update periodically
grep -E '^cpu MHz' /proc/cpuinfo

