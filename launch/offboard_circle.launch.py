from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for offboard.py circle flight:
    - Take off to specified altitude
    - Draw a circle with 1m radius in the air
    - Land
    """
    return LaunchDescription([
        Node(
            package='quadcopter_ctrl',
            executable='offboard.py',
            output='screen',
            parameters=[{
                # ROS / MAVROS basic settings
                'ns': 'mavros',
                'rate_hz': 20.0,
                
                # Flight parameters
                'hover_z': 3.0,           # Takeoff altitude (m)
                'circle_radius': 1.0,      # Circle radius (m)
                'circle_speed': 0.5,       # Circular flight speed (m/s)
                'descent_rate': 0.5,       # Descent speed (m/s)
            }],
        )
    ])

