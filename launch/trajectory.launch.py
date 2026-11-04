from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# EDIT THIS PATH to select the trajectory YAML file.
TRAJECTORY_CONFIG_PATH = '/home/patrick/implicit_swarm/swarm_ws/ros2_ws/src/quadcopter_ctrl/config/trajectory.yaml'  # noqa: E501


def generate_launch_description():
    package_share = get_package_share_directory('quadcopter_ctrl')
    ctrl_launch = package_share + '/launch/ctrl.launch.py'

    include_ctrl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ctrl_launch),
        launch_arguments={
            'config_file': TRAJECTORY_CONFIG_PATH,
        }.items(),
    )

    return LaunchDescription([
        include_ctrl,
    ])
