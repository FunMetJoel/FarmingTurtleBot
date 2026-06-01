from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    drive_robot = LaunchConfiguration('drive_robot')

    return LaunchDescription([
        DeclareLaunchArgument(
            'drive_robot',
            default_value='true',
            description='Set true to run the simple irrigation + movement sim using /cmd_vel'
        ),
        Node(
            package='tb3_field_dt',
            executable='irrigationRoutePlanner',
            name='irrigationRoutePlanner',
            output='screen'
        ),
        Node(
            package='tb3_field_dt',
            executable='simpleRouteFollower',
            name='simpleRouteFollower',
            condition=IfCondition(drive_robot),
            output='screen'
        )
    ])
