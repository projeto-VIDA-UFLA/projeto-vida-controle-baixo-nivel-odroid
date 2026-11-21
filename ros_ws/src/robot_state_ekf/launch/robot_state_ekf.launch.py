"""Sobe o robot_state_ekf_node, que funde GNSS + IMU + encoders + esterçamento em uma estimativa
de estado (x, y, theta, phi, v) via EKF, publicando nav_msgs/Odometry e a tf odom -> base_link.

Uso:
  ros2 launch robot_state_ekf robot_state_ekf.launch.py
  ros2 launch robot_state_ekf robot_state_ekf.launch.py params_file:=/caminho/outro_params.yaml

Para ajustar um parametro pontual sem editar o YAML, use `ros2 run` diretamente com
`--ros-args -p wheelbase:=1.2 -p velocity_source:=front_average`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    params_file_default = (
        get_package_share_directory("robot_state_ekf") + "/config/robot_state_ekf_params.yaml"
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=params_file_default,
        description="Arquivo YAML de parametros do robot_state_ekf_node",
    )

    robot_state_ekf_node = Node(
        package="robot_state_ekf",
        executable="robot_state_ekf_node_exec",
        name="robot_state_ekf_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        params_file_arg,
        robot_state_ekf_node,
    ])
