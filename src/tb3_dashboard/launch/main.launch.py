from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# real robot launch — expects the rest of the stack (Nav2, SLAM, etc) running separately
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock',
        ),
        Node(
            package='tb3_dashboard',
            executable='dashboard_server',
            name='dashboard_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        )
    ])
