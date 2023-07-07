from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    this_pkg = 'zhw_padbot_ops'
    start_padbot_navigation_server_cmd = Node(
        package=this_pkg,
        executable='padbot_navigation_server',
        name='padbot_navigation_server',
        namespace='zhw_padbot_ops',
        parameters=[{'padbot_ip': '192.168.31.5', 'padbot_port': 5000}]
    )
    ld = LaunchDescription()
    ld.add_action(start_padbot_navigation_server_cmd)
    return ld
