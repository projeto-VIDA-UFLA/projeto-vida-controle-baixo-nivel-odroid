from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    package_share = get_package_share_directory('vida_ublox_driver')
    parameters = os.path.join(package_share, 'config', 'ublox_m8u.yaml')

    return LaunchDescription([
        Node(
            package='vida_ublox_driver',
            executable='ublox_m8u_node',
            name='ublox_m8u',
            output='screen',
            parameters=[parameters],
        )
    ])

