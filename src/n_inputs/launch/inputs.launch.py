#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    keyboard_input_node = Node(
        package="n_inputs",
        executable="keyboard_node",
        name="keyboard_node",
        output="screen",
        emulate_tty=True,
        remappings=[
            ("/vel_cmds/keyboard", "/vel_cmds")
        ]
    )

    return LaunchDescription([keyboard_input_node])

