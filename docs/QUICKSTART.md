# Quickstart: run, drive and modify the T-01 sim in ~30 minutes

Every command below is copy-paste. What you should SEE is written after each step.

Gazebo is physics simulation (a digital twin), not VR: gravity, friction, contacts and
sensors are all computed. Every demo video in this repo was filmed inside it.

## 0. One-time setup check

    docker images | findstr tranno_sim

If `tranno_sim` is missing, build it once:

    docker build -t tranno_sim docker/

## 1. Start the sim with the 3D window (Windows + WSLg)

From a **WSL Ubuntu** terminal:

    cd /mnt/d/tranno_robot && bash run.sh

Inside the container:

    cd /ws && colcon build --symlink-install && source install/setup.bash
    ros2 launch tranno_sim sim.launch.py

SEE: the jobsite world: fences with a yard gate, a house, a truck, trees, the orange
drop zone, the cedar stack flat on the ground, and T-01 with its clamp paddles and
camera mast. Press play (bottom left) if physics is paused.

## 2. Drive it by hand

Second terminal:

    docker exec -it tranno_sim bash
    source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash

Keyboard driving (i/j/l/k, x to slow down):

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

The other actuators. Each is a ROS 2 topic: a named channel carrying messages:

    ros2 topic pub --once /lift_cmd std_msgs/msg/Float64 "{data: 0.30}"     # mast up
    ros2 topic pub --once /clamp_l_cmd std_msgs/msg/Float64 "{data: 0.45}"  # close left
    ros2 topic pub --once /clamp_r_cmd std_msgs/msg/Float64 "{data: 0.45}"  # close right
    ros2 topic pub --once /clamp_l_cmd std_msgs/msg/Float64 "{data: 0.0}"   # open
    ros2 topic pub --once /clamp_r_cmd std_msgs/msg/Float64 "{data: 0.0}"

What the robot senses:

    ros2 topic list
    ros2 topic echo /scan --once              # one lidar sweep
    ros2 topic echo /model/t01/pose --once    # true pose

Run the full perception mission (the demo video):

    ros2 run tranno_sim delivery_demo

SEE: the robot spins to find the stack with its camera, docks, hugs, lifts, carries it
through the gate, finds the orange zone by camera, sets it down, backs away. The log
narrates each phase.

## 3. Change the world

The world is one XML file: `ws/src/tranno_sim/worlds/jobsite.sdf`.

The vocabulary (this is 80% of working with Gazebo):

- `<model>`  = a thing in the world (the robot, a tree, the stack)
- `<link>`   = one rigid body inside a model
- `<joint>`  = how links connect and move (`prismatic` slides, `revolute` spins)
- `<sensor>` = camera / lidar, attached to a link
- `<plugin>` = behavior: drivetrain, joint controllers, pose publishers
- `<pose>`   = x y z roll pitch yaw (meters, radians)

Try it: in `<model name="barrel">` change the pose x from 6.7 to 6.0, restart the
launch, and the barrel has moved. That barrel is an unplanned obstacle: put it on the
corridor and watch the lidar avoidance route around it.

Mission logic is one Python control loop: `ws/src/tranno_sim/tranno_sim/delivery_demo.py`.
Read `run()` first: subscribe to pose/camera/lidar, compute an error, publish a velocity.

After editing code or the world, rebuild inside the container (seconds):

    cd /ws && colcon build --symlink-install && source install/setup.bash

## Concept map

    SDF world (jobsite.sdf)      what exists and how it's built
    gz sim (Gazebo Harmonic)     computes physics + renders sensors
    ros_gz bridge                copies messages between Gazebo and ROS 2
    topics (/cmd_vel, /scan...)  the nervous system
    delivery_demo.py             the brain: sense -> decide -> act

That's the whole stack. Everything else is detail.
