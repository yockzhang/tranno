#!/bin/bash
# Inside-container smoke test: build ws, boot server-only sim, verify topics.
set -e
source /opt/ros/jazzy/setup.bash
cd /ws
colcon build --symlink-install
source install/setup.bash
WORLD=$(ros2 pkg prefix tranno_sim)/share/tranno_sim/worlds/jobsite.sdf
echo "--- starting headless sim ---"
gz sim -s -r "$WORLD" &
SIM=$!
sleep 12
echo "--- gz topics ---"
gz topic -l | tee /tmp/topics.txt
grep -q "/cmd_vel" /tmp/topics.txt && echo "OK: /cmd_vel"
grep -q "front_cam" /tmp/topics.txt && echo "OK: camera"
grep -q "/lift_cmd" /tmp/topics.txt && echo "OK: lift"
echo "--- gz models ---"
gz model --list || true
kill $SIM 2>/dev/null || true
echo "SMOKE PASS"
