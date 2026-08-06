#!/bin/bash
# Isolated grip test: dock -> hug -> lift -> report stack height and slip.
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
Z() { timeout 4 ros2 topic echo /model/cedar_stack/pose --once 2>/dev/null | grep -A3 translation | grep -E 'x:|y:|z:' | head -3 | tr '
' ' '; }
echo "stack z at rest: $(Z)"
# drive to straddle (open clamps, creep)
ros2 topic pub --once /clamp_l_cmd std_msgs/msg/Float64 '{data: 0.0}' >/dev/null
ros2 topic pub --once /clamp_r_cmd std_msgs/msg/Float64 '{data: 0.0}' >/dev/null
ros2 topic pub --once /lift_cmd std_msgs/msg/Float64 '{data: 0.05}' >/dev/null
timeout 4 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.28}}' >/dev/null 2>&1
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null
echo "docked. closing clamps..."
ros2 topic pub --once /clamp_l_cmd std_msgs/msg/Float64 "{data: ${1:-0.45}}" >/dev/null
ros2 topic pub --once /clamp_r_cmd std_msgs/msg/Float64 "{data: ${1:-0.45}}" >/dev/null
sleep 3
echo "stack z after hug: $(Z)"
echo "lifting to 0.42..."
ros2 topic pub --once /lift_cmd std_msgs/msg/Float64 '{data: 0.42}' >/dev/null
sleep 3
echo "stack z after lift: $(Z)"
sleep 4
echo "stack z 4s later (slip check): $(Z)"
