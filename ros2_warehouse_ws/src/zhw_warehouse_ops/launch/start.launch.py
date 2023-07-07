from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess


def generate_launch_description():
    HOME = os.getenv('HOME')
    # # jaka ros1 driver
    # start_jaka_ros1_driver = ExecuteProcess(
    #     cmd=['bash', '-c', f"cd {HOME}/ros_code/ros1_jaka_ws && ./start.bash"],
    #     output='screen'
    # )

    # include ros2 launch files
    launch_jaka_ops_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('zhw_jaka_ops'), 'launch', 'start.launch.py')
        )
    )
    launch_padbot_ops_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('zhw_padbot_ops'), 'launch', 'start.launch.py')
        )
    )

    # nodes
    start_pick_objct_by_name_server_cmd = Node(
        package='zhw_warehouse_ops',
        executable='pick_object_by_name_server',
        name='pick_object_by_name_server',
        output='screen',
        namespace='zhw_warehouse_ops'
    )

    ld = LaunchDescription()
    # ld.add_action(start_jaka_ros1_driver)
    ld.add_action(launch_jaka_ops_cmd)
    ld.add_action(launch_padbot_ops_cmd)
    ld.add_action(start_pick_objct_by_name_server_cmd)
    return ld
