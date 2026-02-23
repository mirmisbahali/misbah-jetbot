"""
Teleop bringup launch file — Host laptop side.

Starts:
  1. joy_node             — reads PS4 controller via /dev/input/js0
  2. teleop_twist_joy     — converts Joy messages to Twist velocity commands

The cmd_vel output is remapped to /diff_drive_controller/cmd_vel_unstamped
so it reaches the diff_drive_controller on the Jetson Nano via ROS2 DDS.

Prerequisites:
  - PS4 controller paired via Bluetooth (bluetoothctl)
  - Both machines share the same ROS_DOMAIN_ID (set in devcontainer.json)
  - Both machines on the same WiFi subnet

Usage:
  ros2 launch teleop_bringup teleop.launch.py
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joy_params = PathJoinSubstitution(
        [FindPackageShare("teleop_bringup"), "config", "joy_params.yaml"]
    )
    teleop_params = PathJoinSubstitution(
        [FindPackageShare("teleop_bringup"), "config", "ps4_teleop.yaml"]
    )

    # 1. Joy node — reads raw joystick events from /dev/input/js0
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[joy_params],
        output="screen",
    )

    # 2. Teleop twist joy — converts joy to Twist, remapped to diff_drive topic
    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[teleop_params],
        remappings=[
            # teleop_twist_joy publishes to /cmd_vel by default;
            # remap to the topic diff_drive_controller subscribes to
            ("/cmd_vel", "/diff_drive_controller/cmd_vel_unstamped"),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            joy_node,
            teleop_node,
        ]
    )
