# Founder tour: run, drive and modify the T-01 sim in ~30 minutes

Every command below is copy-paste. Do them in order. What you should SEE is written
after each step, so you know it worked.

Gazebo is physics simulation (a digital twin), not VR: gravity, friction, contacts and
sensors are all computed. Every demo video we have was filmed inside it.

## 0. One-time setup check (5 min)

Open PowerShell:

    docker images | findstr tranno_sim

You should see `tranno_sim`. If not, build it once (takes a while):

    cd D:\tranno_robot
    docker build -t tranno_sim docker/

## 1. Start the sim WITH the 3D window (WSLg) (5 min)

Open a **WSL Ubuntu** terminal (not PowerShell), then:

    cd /mnt/d/tranno_robot && bash run.sh

Inside the container that opens:

    cd /ws && colcon build --symlink-install && source install/setup.bash
    ros2 launch tranno_sim sim.launch.py

SEE: a Gazebo window with the jobsite: grey ground, brown fences with a gate, a house,
a truck, trees, the orange drop zone, the cedar stack on the ground, and T-01 with its
yellow clamp paddles and camera mast. Press the orange play button (bottom left) if
physics is paused.

## 2. Drive it yourself (10 min)

Second terminal into the same container:

    docker exec -it tranno_sim bash
    source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash

Drive with the keyboard (i/j/l/k keys, x to slow down):

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

Now the robot's other muscles. Each of these is a "topic": a named channel that carries
messages. Topics are the robot's nervous system.

    # raise the mast
    ros2 topic pub --once /lift_cmd std_msgs/msg/Float64 "{data: 0.30}"
    # close the clamp paddles
    ros2 topic pub --once /clamp_l_cmd std_msgs/msg/Float64 "{data: 0.45}"
    ros2 topic pub --once /clamp_r_cmd std_msgs/msg/Float64 "{data: 0.45}"
    # open them again
    ros2 topic pub --once /clamp_l_cmd std_msgs/msg/Float64 "{data: 0.0}"
    ros2 topic pub --once /clamp_r_cmd std_msgs/msg/Float64 "{data: 0.0}"

Peek at what the robot senses:

    ros2 topic list                 # everything on the bus
    ros2 topic echo /scan --once    # one lidar sweep (480 ranges in meters)
    ros2 topic echo /model/t01/pose --once   # where the robot truly is

Try the whole delivery mission (the thing in the demo videos):

    ros2 run tranno_sim delivery_demo

SEE: the robot spins to find the stack with its camera, docks, hugs, lifts, carries it
through the gate, finds the orange zone, sets it down, backs away. The log narrates
every phase.

## 3. Change the world (10 min)

The world is one XML file. Open it in any editor ON WINDOWS:

    D:\tranno_robot\ws\src\tranno_sim\worlds\jobsite.sdf

The vocabulary (this is 80% of Gazebo):

- `<model>`  = a thing in the world (the robot, a tree, the stack)
- `<link>`   = one rigid body inside a model
- `<joint>`  = how two links connect and move (our lift is `prismatic` = slides;
  wheels are `revolute` = spin)
- `<sensor>` = camera / lidar, attached to a link
- `<plugin>` = behavior: motors (DiffDrive), joint controllers, pose publishers
- `<pose>`   = x y z roll pitch yaw, meters and radians

Try it: find `<model name="barrel">` and change its pose x from 6.7 to 6.0. Ctrl-C the
sim launch, relaunch it, and the red barrel has moved. That barrel is the unplanned
obstacle the lidar avoidance dodges: move it onto the corridor and watch the mission
route around it.

The mission logic lives in one Python file:

    D:\tranno_robot\ws\src\tranno_sim\tranno_sim\delivery_demo.py

Read `run()` bottom-up: it is a plain control loop: subscribe to pose/camera/lidar,
compute an error, publish a velocity. Nothing mystical.

After editing PYTHON or the WORLD, rebuild once inside the container (fast):

    cd /ws && colcon build --symlink-install && source install/setup.bash

## 4. Where the bodies are buried

`tasks/lessons.md` is the honest log of every mistake this sim taught us: contact
physics, sensor placement, load retention, stale-pose ghosts. Read it before you trust
any demo, ours included.

## Concept map (60 seconds)

    SDF world (jobsite.sdf)      what exists and how it's built
    gz sim (Gazebo Harmonic)     computes physics + renders sensors
    ros_gz bridge                copies messages between Gazebo and ROS 2
    topics (/cmd_vel, /scan...)  the nervous system
    delivery_demo.py             the brain: senses -> decides -> acts

That's the whole stack. Everything else is detail.
