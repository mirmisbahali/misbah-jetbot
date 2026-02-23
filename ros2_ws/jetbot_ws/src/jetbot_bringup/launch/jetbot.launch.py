"""
JetBot bringup launch file — Jetson Nano side.

Starts:
  1. robot_state_publisher   — publishes TF from URDF + joint states
  2. ros2_control_node       — controller_manager, loads hardware plugin
  3. joint_state_broadcaster — spawned immediately after controller_manager
  4. diff_drive_controller   — spawned after joint_state_broadcaster is active

Topic for driving the robot:
  /diff_drive_controller/cmd_vel_unstamped  (geometry_msgs/Twist)

Usage:
  ros2 launch jetbot_bringup jetbot.launch.py
  ros2 launch jetbot_bringup jetbot.launch.py port:=/dev/ttyTHS1 max_wheel_speed_rad_s:=12.0
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Launch arguments ────────────────────────────────────────────────────
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyTHS1",
        description="Serial port connected to the Raspberry Pi Pico",
    )
    baud_arg = DeclareLaunchArgument(
        "baud_rate",
        default_value="115200",
        description="UART baud rate",
    )
    max_speed_arg = DeclareLaunchArgument(
        "max_wheel_speed_rad_s",
        default_value="15.0",
        description="Maximum wheel angular velocity in rad/s (used for normalisation)",
    )

    port = LaunchConfiguration("port")
    baud_rate = LaunchConfiguration("baud_rate")
    max_wheel_speed_rad_s = LaunchConfiguration("max_wheel_speed_rad_s")

    # ── Robot description (URDF via xacro) ─────────────────────────────────
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("robot_description"), "urdf", "jetbot.urdf.xacro"]
            ),
            " port:=", port,
            " baud_rate:=", baud_rate,
            " max_wheel_speed_rad_s:=", max_wheel_speed_rad_s,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # ── Controller config ───────────────────────────────────────────────────
    robot_controllers_path = os.path.join(
        get_package_share_directory("robot_description"), "config", "jetbot_controllers.yaml"
    )
    with open(robot_controllers_path, "r") as f:
        robot_controllers = yaml.safe_load(f)

    # ── Nodes ───────────────────────────────────────────────────────────────

    # 1. Publishes /tf from URDF + /joint_states
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # 2. Controller manager — loads hardware plugin via pluginlib
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
    )

    # 3. Joint state broadcaster — publishes /joint_states
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 4. Differential drive controller — spawned after broadcaster is ready
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawn diff_drive_controller only after joint_state_broadcaster has exited
    # (spawner exits with code 0 once the controller is active)
    delay_diff_drive_after_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            port_arg,
            baud_arg,
            max_speed_arg,
            robot_state_publisher_node,
            controller_manager_node,
            joint_state_broadcaster_spawner,
            delay_diff_drive_after_broadcaster,
        ]
    )
