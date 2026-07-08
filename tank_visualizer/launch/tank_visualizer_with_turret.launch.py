from launch import LaunchDescription
from launch_ros.actions import Node  # <-- This was missing!
import os

def generate_launch_description():
    # Direct path pointing exactly to your live Python node scripts
    base_path = os.path.expanduser('~/ros2_ws/src/ROS2-Tank-Visualization/tank_visualizer/tank_visualizer')
    
    body_script = os.path.join(base_path, 'tank_body_node.py')
    turret_script = os.path.join(base_path, 'turret_controller.py')

    return LaunchDescription([
        # Run the body node script directly via python3
        Node(
            executable='python3',
            arguments=[body_script],
            name='tank_body_visualizer',
            output='screen',
            emulate_tty=True,
        ),
        # Run the turret node script directly via python3
        Node(
            executable='python3',
            arguments=[turret_script],
            name='turret_controller',
            output='screen',
            emulate_tty=True,
        ),
    ])