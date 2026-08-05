"""Launch the jobsite world in Gazebo + ROS bridges.

  ros2 launch tranno_sim sim.launch.py            # with GUI (needs display / WSLg)
  ros2 launch tranno_sim sim.launch.py headless:=true
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world = os.path.join(
        get_package_share_directory('tranno_sim'), 'worlds', 'jobsite.sdf')

    headless = LaunchConfiguration('headless')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),

        # Gazebo (GUI)
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world],
            output='screen',
            condition=UnlessCondition(headless)),
        # Gazebo (server only)
        ExecuteProcess(
            cmd=['gz', 'sim', '-s', '-r', world],
            output='screen',
            condition=IfCondition(headless)),

        # ROS <-> Gazebo bridges
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/lift_cmd@std_msgs/msg/Float64@gz.msgs.Double',
                '/tilt_cmd@std_msgs/msg/Float64@gz.msgs.Double',
                '/detach@std_msgs/msg/Empty@gz.msgs.Empty',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/model/t01/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                '/front_cam@sensor_msgs/msg/Image@gz.msgs.Image',
            ],
            output='screen'),
    ])
