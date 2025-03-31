#!/usr/bin/env python3
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    odrive_node = Node(
        package="n_odrive",
        executable="odrive_node",
        name="odrive_node",
        output="screen",
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("n_controller"), "launch", "motors.launch.py"]
                ),
            ]
        ),
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("n_imu"), "launch", "imu.launch.py"]
                ),
            ]
        ),
    )

    # the direct keyboard node is launched, because we need direct access to the keyboard input
    keyboard_input_node = Node(
        package="n_inputs",
        executable="keyboard_node",
        name="keyboard_node",
        output="screen",
        emulate_tty=True,
        remappings=[("/cmd_vel/keyboard", "/cmd_vel")],
    )
    
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    gamepad_input_node = Node(
        package="n_inputs",
        executable="gamepad_node",
        name="gamepad_node",
        output="screen",
        emulate_tty=True,
        remappings=[("/cmd_vel/gamepad", "/cmd_vel")],
    )

    return LaunchDescription(
        [
            odrive_node,
            controller_launch,
            imu_launch,
            # keyboard_input_node,
            joy_node,
            gamepad_input_node,
        ]
    )
