import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    web_dir = os.path.join(get_package_share_directory('tb3_dashboard'), 'web')

    return LaunchDescription([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{'port': 9090}],
            output='log',
        ),
        Node(
            package='tb3_dashboard',
            executable='dashboard_bridge',
            name='dashboard_bridge',
            output='log',
        ),
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8080'],
            cwd=web_dir,
            output='log',
        ),
    ])
