import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ublox_config = os.path.join(
        get_package_share_directory('vida_ublox_driver'),
        'config',
        'ublox_m8u.yaml',
    )

    return LaunchDescription([
        Node(
            package='vida_ublox_driver',
            executable='ublox_m8u_node',
            name='ublox_m8u',
            namespace='vida',
            output='screen',
            parameters=[ublox_config],
        ),
        Node(
            package='vida_odroid',
            executable='security',
            name='security',
            output='screen',
        ),
    ])
