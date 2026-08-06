# Tranno T-01 simulator

Simulation for T-01, a driverless tracked robot that delivers material on residential
jobsites. It carries a 60 to 80 kg stack (lumber, pavers, cement bags) across the ground a
truck can't reach: mud, gravel, slopes, 90 cm gates. The goal is to replace the manual
material running that eats up to 42% of crew time.

We simulate first. Every mechanism and control decision runs in Gazebo before any hardware
money is spent. Real T-01 v1 is a retrofit: an off-the-shelf tracked carrier with the
operator station removed, plus an electric heavy-payload arm, lift deck, cameras and RTK-GPS.

Built with ROS 2 Jazzy and Gazebo Harmonic, fully inside Docker. No local ROS install needed.

## Watch it run

67 seconds, fixed overview camera with the robot's own view inset:
[docs/Tranno-Sim-Demo.mp4](docs/Tranno-Sim-Demo.mp4). Mission log from the same run:
[docs/demo_mission.log](docs/demo_mission.log). One take (middle section labeled 3x),
labeled simulation. BEAR-HUG loading, not a forklift and not an arm: the 60 kg stack
lies flat on the bare ground (no pallet, no dunnage). The robot straddles it with two
clamp paddles, hugs it (force-limited squeeze), hops it up to seat the underhook toes,
lifts, carries it through the yard gate on rough ground, sets it down centered on the
drop zone, opens up, raises the paddles clear over the top and backs away. Verified by
telemetry, not by eyeballing: the stack rides at z=0.4 m the whole route and lands
6 cm from the zone center. Load and unload are the same mechanism reversed.

## What's in the sim

- `ws/src/tranno_sim/worlds/jobsite.sdf` : the world. Rough ground with bumps, a material
  pile, a scrap-wood obstacle, an orange drop zone, and the robot.
- The robot: skid-steer base (stands in for tracks), a front mast with a prismatic lift,
  and a BEAR-HUG clamp: two paddles on lateral prismatic joints squeeze the load
  (force-limited) and underhook toes at the paddle bottoms seat beneath it after a
  small hop, so the load rides on mechanical support, not just friction. The 60 kg
  cedar stack is a free body flat on the ground: grabbed, carried and placed by
  contact physics alone, no attach cheats, no pallets.
- `launch/sim.launch.py` : starts Gazebo and bridges the topics to ROS 2:
  `/cmd_vel`, `/lift_cmd`, `/tilt_cmd`, `/detach`, `/odom`, `/front_cam`.
- `tranno_sim/delivery_demo.py` : the full mission. Closed-loop waypoint navigation on
  odometry, around the obstacle to the drop zone, then lower, tilt, release, back away.

## Run it

Build the image once:

    docker build -t tranno_sim docker/

With GUI (Windows, WSLg), from a WSL Ubuntu terminal:

    bash run.sh
    # inside the container:
    cd /ws && colcon build --symlink-install && source install/setup.bash
    ros2 launch tranno_sim sim.launch.py

Run the delivery mission in a second terminal (`docker exec -it tranno_sim bash`):

    source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
    ros2 run tranno_sim delivery_demo

Drive it yourself instead:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ros2 topic pub --once /lift_cmd std_msgs/msg/Float64 "{data: 0.3}"
    ros2 topic pub --once /tilt_cmd std_msgs/msg/Float64 "{data: 0.6}"
    ros2 topic pub --once /detach std_msgs/msg/Empty "{}"

Headless smoke test (no display):

    docker run --rm -v <repo>/ws:/ws -v <repo>/scripts:/scripts tranno_sim bash /scripts/smoke.sh

## What the sim has already taught us

Two findings changed the hardware spec before we bought anything:

1. Skid-steer dead-reckoning drifted about 1.5 m over one short mission. Vision plus wheel
   odometry is not enough outdoors. The real machine gets fused RTK-GPS, inertial and visual
   localization.
2. Releasing the payload joint did nothing: friction kept the stack on the deck. Unloading
   needs a mechanism. The deck now tilts to dump, and the hardware spec carries the same
   requirement.

## Status

Sim runs the full pile-to-drop-zone mission. Next: ground-truth odometry plugin, tighter
drop placement, pile self-loading with the arm, and a recorded sim demo. Sim footage is
always labeled as sim; it is never passed off as real hardware.

More at [gotranno.com](https://gotranno.com).
