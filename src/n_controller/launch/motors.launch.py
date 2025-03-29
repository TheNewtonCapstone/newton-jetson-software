#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    shared_directory = get_package_share_directory("n_controller")
    ros_config_path = os.path.join(
        shared_directory,
        "config",
        "onnx.yaml",
    )

    with open(ros_config_path, "r") as file:
        onnx_config = yaml.safe_load(file)

    relative_model_path = onnx_config["model_path"]

    onnx_config["model_path"] = os.path.join(
        shared_directory,
        relative_model_path,
    )

    print(
        f"Switched model path from {relative_model_path} to {onnx_config['model_path']}"
    )

    controller_node = Node(
        package="n_controller",
        executable="controller_node",
        name="controller_node",
        output="screen",
        emulate_tty=True,
        parameters=[onnx_config],
    )

    return LaunchDescription([controller_node])
