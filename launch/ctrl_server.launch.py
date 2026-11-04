from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Change this default to the desired hover altitude, in metres.
    hover_z_arg = DeclareLaunchArgument(
        'hover_z',
        default_value='0.25',
        description='Absolute local-frame Z altitude used for takeoff and hover (m)',
    )

    ctrl_server = Node(
        package='quadcopter_ctrl',
        executable='ctrl_server_node',
        name='CTRL_SERVER',
        output='screen',
        parameters=[{
            'hover_z': ParameterValue(
                LaunchConfiguration('hover_z'),
                value_type=float,
            ),
        }],
    )

    return LaunchDescription([
        hover_z_arg,
        ctrl_server,
    ])
