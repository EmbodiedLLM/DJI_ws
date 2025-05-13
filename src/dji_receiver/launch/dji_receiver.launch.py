from launch import LaunchDescription
from launch_ros.actions import Node
import os

CONFIG_PATH = os.path.join(
    os.path.dirname(__file__), '..', 'config', 'params.yaml')


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dji_receiver',
            executable='dji_receiver_node',
            name='dji_receiver_node',
            output='screen',
            parameters=[CONFIG_PATH],
        )
    ]) 