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

92 seconds, fixed overview camera with the robot's masthead view inset:
[docs/Tranno-Sim-Demo.mp4](docs/Tranno-Sim-Demo.mp4). Mission log from the same run:
[docs/demo_mission.log](docs/demo_mission.log). One take (sped-up sections labeled),
labeled simulation, with on-screen phase captions. PERCEPTION-DRIVEN: nothing about
the pile or the drop point is hardcoded. The robot SCANS with its masthead camera and
finds the cedar stack (color segmentation), visual-servos in, bear-hugs it (force
squeeze + underhook toes), carries it on the site corridor with a 2D lidar watching
for unplanned obstacles, threads the yard gate closed-loop, then finds the ORANGE drop
zone by camera, servos to it and sets the load down. Load telemetry verifies every
phase (the load's own z is the only honest metric). Honest gaps, on the record: the
load can still shed at the gate exit on some runs (~50%), placement lands within
~0.9 m of the zone center, and the route corridor + gate line still come from the site
map. Next: tighter placement, multi-trip loop, drop-verification (Milestone 2).

## Quickstart

[docs/QUICKSTART.md](docs/QUICKSTART.md): a 30-minute copy-paste tour: run the sim with
the 3D window, drive the robot by hand, work the clamp, run the full perception
mission, and make your first world edit.

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
