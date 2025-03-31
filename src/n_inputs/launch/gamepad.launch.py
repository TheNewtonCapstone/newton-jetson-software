#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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
        remappings=[
            ("/cmd_vel/keyboard", "/cmd_vel")
        ]
    )

    return LaunchDescription([joy_node, gamepad_input_node])

