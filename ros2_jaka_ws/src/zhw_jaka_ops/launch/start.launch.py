from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    this_pkg = 'zhw_jaka_ops'
    # included_launch = IncludeLaunchDescription(
    #     package='zhw_jaka_wrapper',
    #     launch='start.py'
    # )
    included_launch = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('zhw_jaka_wrapper'),
                    'launch',
                    'start.launch.py'
                ])
            ]),
            launch_arguments={}.items()
        )])
    shelf2cart_by_name = Node(
        package=this_pkg,
        executable='shelf2cart_by_name',
        name='shelf2cart_by_name',
        namespace=this_pkg
    )
    return LaunchDescription([shelf2cart_by_name, included_launch])
