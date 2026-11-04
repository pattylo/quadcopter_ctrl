import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('quadcopter_ctrl'),
        'config',
        'trajectory.yaml',
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='YAML file containing controller and trajectory parameters',
    )

    ctrl_server = Node(
        package='quadcopter_ctrl',
        executable='ctrl_server_node',
        name='CTRL_SERVER',
        output='screen',
        parameters=[ParameterFile(
            LaunchConfiguration('config_file'),
            allow_substs=True,
        )],
    )

    return LaunchDescription([
        config_file_arg,
        ctrl_server,
    ])
