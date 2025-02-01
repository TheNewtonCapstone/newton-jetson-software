from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os
import yaml

def generate_launch_description():
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
    ros_params = {}

    for motor_name, motor_config in motors_params.items():
        for key, value in motor_config.items():
            ros_params[f"m_{motor_name.lower()}_{key}"] = f"{str(value).lower()}"

    for key, value in ros_params.items():
        print(f"{key} : {value}")


    return LaunchDescription([
        Node(
            package="parameters",
            executable="param_node",
            name="custom_minimal_param_node",
            output="screen",
            emulate_tty=True,
            parameters=[ros_params]
        )
    ])