from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    this_dir = os.path.dirname(os.path.realpath(__file__))
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        Node(
            package='tb3_state_dt',
            executable='rob_water_level',
            name='rob_water_level',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='tb3_state_dt',
            executable='sim_water_level',
            name='sim_water_level',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='tb3_state_dt',
            executable='sim_battery_level',
            name='sim_battery_level',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='tb3_state_dt',
            executable='rob_state',
            name='rob_state',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='tb3_state_dt',
            executable='sim_state',
            name='sim_state',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(this_dir, 'gazebo_twin.launch.py')
            )
        )
    ])

