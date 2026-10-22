import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    f9r_rtk_launch = os.path.join(
        get_package_share_directory('vida_ntrip_client'),
        'launch',
        'zed_f9r_rtk.launch.py',
    )

    # EKF (robot_state_ekf) no Odroid: os sensores (GPS/IMU, encoders e LVDT pelo
    # micro-ROS) sao locais; o notebook recebe so odom e status.
    ekf_launch = os.path.join(
        get_package_share_directory('robot_state_ekf'),
        'launch',
        'robot_state_ekf.launch.py',
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f9r_rtk_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_launch),
        ),
        Node(
            package='vida_odroid',
            executable='security',
            name='security',
            output='screen',
        ),
    ])
