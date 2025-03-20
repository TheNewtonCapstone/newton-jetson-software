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
    shared_directory = get_package_share_directory("n_controller")
    robot_config_path = os.path.join(
        shared_directory,
        "config", 
        "robot.yaml"
    )

    try:
        if not os.path.exists(robot_config_path):
            raise FileNotFoundError
        with open(robot_config_path, "r") as file:
            configs = yaml.safe_load(file)
    except FileNotFoundError:
        print(f"Configuration file not found: {robot_config_path}")

    motors_params = configs["motors_params"]
    # Declare all launch arguments
    ros_motor_config = {}
    ros_motor_config["robot_name"] = configs["robot_name"]
    ros_motor_config["num_joints"] = len(motors_params)
    print(f"Number of joints: {ros_motor_config['num_joints']}")

    for motor_name, motor_config in motors_params.items():
        for key, value in motor_config.items():
            param_name = f"{motor_name.lower()}_{key}"
            if isinstance(value, (int, float)):
                ros_motor_config[param_name] = float(value)
            else:
                ros_motor_config[param_name] = str(value)

    nodes = []

    for motor_name, motor_config in motors_params.items():
        print(f"Motor name: {motor_name}")
        print(f"Motor config: {motor_config}")
        nodes.append(create_motor_node(motor_name, motor_config))
        
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
    
    print(f"Switched model path from {relative_model_path} to {onnx_config['model_path']}")
    
    config = {}
    config.update(ros_motor_config)
    config.update(onnx_config)

    controller_node = Node(
        package="n_controller",
        executable="controller_node",
        name="controller_node",
        output="screen",
        emulate_tty=True,
        parameters=[config],
    )

    return LaunchDescription([*nodes, controller_node])
