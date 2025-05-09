import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('n_imu'),
        'config',
        'params.yaml'
    )

    imu_node = Node(
        package="n_imu",
        executable="transducer_m_imu",
        name="n_imu",
        output="screen",
        parameters=[config],
    )

    return LaunchDescription([imu_node])
