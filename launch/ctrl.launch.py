import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('quadcopter_ctrl'),
        'launch',
        'trajectory.yaml',
    )

    ctrl_server = Node(
        package='quadcopter_ctrl',
        executable='ctrl_server_node',
        name='CTRL_SERVER',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([ctrl_server])
