from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{'port': 9090}],
            output='screen',
        ),
        Node(
            package='tb3_dashboard',
            executable='dummy_publisher',
            name='dummy_publisher',
            output='screen',
        ),
    ])
