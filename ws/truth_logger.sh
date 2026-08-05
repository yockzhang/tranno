#!/bin/bash
source /opt/ros/jazzy/setup.bash
export GZ_VERSION=harmonic
for i in $(seq 1 90); do
  echo "$i $(gz model -m t01 -p 2>/dev/null | grep -A1 XYZ | tail -1)"
  sleep 2
done > /tmp/truth.log
