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
    print(f"Creating motor node for {motor_config['node_id']}")
    node = Node(
        package="odrive_can",
        executable="odrive_can_node",
        name="odrive_can",
        namespace=f"{motor_name.lower()}",
        parameters=[
            {
                "node_id": motor_config["node_id"],
                "interface": "can0",
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
    root_path = pythonpath = os.environ.get("NEWTON_ROOT")
    config_path = os.path.join(root_path, "configs", "newton.yaml")

    try:
        if not os.path.exists(config_path):
            raise FileNotFoundError
        with open(config_path, "r") as file:
            configs = yaml.safe_load(file)
    except FileNotFoundError:
        print(f"Configuration file not found: {config_path}")

    motors_params = configs["motors_params"]
    # Declare all launch arguments
    launch_args = {}
    for motor_name, motor_config in motors_params.items():
        for key, value in motor_config.items():
            launch_args[f"m_{motor_name.lower()}_{key}"] = f"{str(value).lower()}"

    nodes = []

    for motor_name, motor_config in motors_params.items():
        print(f"Motor name: {motor_name}")
        print(f"Motor config: {motor_config}")
        nodes.append(create_motor_node(motor_name, motor_config))

    motor_controller_node =  Node (
        package="motor_driver",
        executable="motor_driver_node",
        name="motor_driver_node",
        output="screen",
        emulate_tty=True,
        parameters=[launch_args],
    )

    return LaunchDescription([*nodes, motor_controller_node])
