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
    # micro-ROS) sao locais; o notebook recebe so odom e status. Parametros do
    # YAML do pacote, com os ajustes do VIDA por cima.
    ekf_params = os.path.join(
        get_package_share_directory('robot_state_ekf'),
        'config',
        'robot_state_ekf_params.yaml',
    )
    ekf_overrides = {
        # Espera o veiculo andar (2 m com RTK FIXED) para tirar o heading do
        # GPS, sem limite pratico. Com os 20 s do pacote, ligar o sistema e
        # demorar a sair fazia o EKF iniciar com o heading do parametro
        # (errado) - teste de 30/10.
        'heading_init_timeout_sec': 86400.0,
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f9r_rtk_launch),
        ),
        Node(
            package='robot_state_ekf',
            executable='robot_state_ekf_node_exec',
            name='robot_state_ekf_node',
            output='screen',
            parameters=[ekf_params, ekf_overrides],
        ),
        Node(
            package='vida_odroid',
            executable='security',
            name='security',
            output='screen',
        ),
    ])
