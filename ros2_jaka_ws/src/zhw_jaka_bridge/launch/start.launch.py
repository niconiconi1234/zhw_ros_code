from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='zhw_jaka_bridge',
            executable='bridge',
            name='zhw_jaka_bridge',
        )
    ])
