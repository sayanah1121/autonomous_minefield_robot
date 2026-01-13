from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='minefield_bot',
            executable='hardware_bridge',
            name='hardware_bridge',
            output='screen'
        ),
        Node(
            package='minefield_bot',
            executable='mapper',
            name='mapper',
            output='screen'
        ),
        Node(
            package='minefield_bot',
            executable='planner',
            name='planner',
            output='screen'
        ),
    ])
