#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import yaml


def create_motor_node(motor_name, motor_config):
    node = Node(
        package="odrive_can",
        executable="odrive_can_node",
        name=f"can_node_{motor_name.lower()}",
        namespace=f"{motor_name.lower()}",
        parameters=[
            {
                "node_id": motor_config["node_id"],
                "can_interface": "can0",
                "publish_period_ms": 1,
                "axis_idle_on_shutdown": True,
            }
        ],
        remappings=[
            ("control_message", "control_message"),
            ("odrive_status", "odrive_status"),
            ("controller_status", "controller_status"),
        ],
    )
    return node


def generate_launch_description():
    # get root path of the package
    use_imu = DeclareLaunchArgument(
        "imu",
        default_value="true",
        description="Whether to use the IMU",
    )

    use_motors = DeclareLaunchArgument(
        "motors",
        default_value="true",
        description="Whether to use the motors",
    )

    imu_pkg_dir = FindPackageShare("tm_imu")
    motor_pkg_dir = FindPackageShare("motor_driver")

    print(f"IMU package dir: {imu_pkg_dir}")
    print(f"Motor package dir: {motor_pkg_dir}")

    imu_launch = IncludeLaunchDescription(
        PathJoinSubstitution([imu_pkg_dir, "launch", "imu.launch.py"])
    )
    motor_launch = IncludeLaunchDescription(
        PathJoinSubstitution([motor_pkg_dir, "launch", "motors.launch.py"])
    )

    # can_interface = DeclareLaunchArgument(
    #     "can_interface",
    #     default_value="can0",
    #     description="CAN interface to use",
    # )

    return LaunchDescription([use_imu, use_motors, imu_launch, motor_launch])
