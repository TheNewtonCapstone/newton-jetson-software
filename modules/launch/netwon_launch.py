#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare all launch arguments
    launch_args = []

    imu_pkg_dir = FindPackageShare('tm_imu')

    imu_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            imu_pkg_dir,
            'launch',
            'imu.launch.py'
        ])
    )

    odrive_node_0 = Node(
        package='odrive_can',
        executable='odrive_can_node',
        name='odrive_can_node',
        output='screen',
        parameters=[{
            'can_interface': 'can0',
            'node_id': 0,
            'publish_period_ms': 1
        }]
    )

    odrive_node_1 = Node(
        package='odrive_can',
        executable='odrive_can_node',
        name='odrive_can_node',
        output='screen',
        parameters=[{
            'can_interface': 'can0',
            'node_id': 1,
            'publish_period_ms': 1
        }]
    )

    motor_node_0 = Node(
        package='odrive_motor',
        executable='odrive_motor_node',
        name='motor_controller_0',
        output='screen',
        parameters=[{
            'control_mode': 'velocity',
            'max_velocity': 10.0,
            'max_current': 10.0
        }]
    )

    motor_node_1 = Node(
        package='odrive_motor',
        executable='odrive_motor_node',
        name='motor_controller_1',
        output='screen',
        parameters=[{
            'control_mode': 'velocity',
            'max_velocity': 10.0,
            'max_current': 10.0
        }]
    )

    ld = LaunchDescription(launch_args)

    ld.add_action(imu_launch)
    ld.add_action(odrive_node_0)
    ld.add_action(odrive_node_1)
    ld.add_action(motor_node_0)
    ld.add_action(motor_node_1)

    return ld
