from launch import LaunchDescription
from launch_ros.actions import Node

# offline demo — no robot or sim needed, dummy_publisher fakes all topics
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tb3_dashboard',
            executable='dashboard_server',
            name='dashboard_server',
            output='screen',
        ),
        Node(
            package='tb3_dashboard',
            executable='dummy_publisher',
            name='dummy_publisher',
            output='screen',
        ),
    ])
