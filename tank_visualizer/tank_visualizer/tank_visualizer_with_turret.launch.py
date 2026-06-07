from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tank_visualizer',
            executable='tank_body_node',
            name='tank_body_visualizer',
            output='screen',
            emulate_tty=True,
        ),
    ])
