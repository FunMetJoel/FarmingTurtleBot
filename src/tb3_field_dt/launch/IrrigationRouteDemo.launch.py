from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    drive_robot = LaunchConfiguration('drive_robot')
    simulate_water = LaunchConfiguration('simulate_water')
    initial_water_level = LaunchConfiguration('initial_water_level')
    start_safety_supervisor = LaunchConfiguration('start_safety_supervisor')

    return LaunchDescription([
        DeclareLaunchArgument(
            'drive_robot',
            default_value='true',
            description='Set true to run the simple irrigation + movement sim using /cmd_vel'
        ),
        DeclareLaunchArgument(
            'simulate_water',
            default_value='true',
            description='Start the simulated robot water tank'
        ),
        DeclareLaunchArgument(
            'initial_water_level',
            default_value='1.0',
            description='Starting simulated tank level from 0.0 to 1.0'
        ),
        DeclareLaunchArgument(
            'start_safety_supervisor',
            default_value='true',
            description='Start safety supervisor if one is not already running'
        ),
        Node(
            package='tb3_state_dt',
            executable='rob_water_level',
            name='rob_water_level',
            condition=IfCondition(simulate_water),
            parameters=[{
                'initial_water_level': initial_water_level
            }],
            output='screen'
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
        ),
        Node(
            package='tb3_navigation_dt',
            executable='safetySupervisorNode',
            name='safetySupervisorNode',
            condition=IfCondition(start_safety_supervisor),
            output='screen'
        )
    ])
