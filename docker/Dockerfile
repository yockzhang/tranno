# Tranno robot sim: ROS 2 Jazzy + Gazebo Harmonic (included in desktop-full)
FROM osrf/ros:jazzy-desktop-full

# Tools + colcon (idempotent if already present)
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    ros-jazzy-ros-gz \
    ros-jazzy-teleop-twist-keyboard \
    nano tmux \
    && rm -rf /var/lib/apt/lists/*

# Workspace is bind-mounted at /ws
WORKDIR /ws

# Source ROS on every shell
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc

CMD ["bash"]
