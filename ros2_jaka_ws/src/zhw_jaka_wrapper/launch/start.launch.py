from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    this_pkg = 'zhw_jaka_wrapper'
    move_line_wrapper = Node(
        package=this_pkg,
        executable='move_line_wrapper',
        name='move_line_wrapper',
        namespace=this_pkg,
        parameters=[
            {'move_vel': 0.5, 'move_acc': 0.3},
        ]
    )
    ag95_gripper_wrapper = Node(
        package=this_pkg,
        executable='ag95_gripper_wrapper',
        name='ag95_gripper_wrapper',
        namespace=this_pkg,
        parameters=[
            {'ag95_gripper_ip': '192.168.31.29'},
            {'ag95_gripper_port': 8888},

        ]
    )
    jaka_bridge = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('zhw_jaka_bridge'),
                    'launch',
                    'start.launch.py'
                ])
            ]),
            launch_arguments={}.items()
        )])
    return LaunchDescription([move_line_wrapper, ag95_gripper_wrapper, jaka_bridge])
