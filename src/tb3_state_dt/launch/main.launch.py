from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os



def generate_launch_description():
    this_dir = os.path.dirname(os.path.realpath(__file__))

    return LaunchDescription([
        Node(
            package='tb3_state_dt',
            executable='orchestrator',
            name='orchestrator',
            output='screen',
        ),
        Node(
            package='tb3_state_dt',
            executable='rob_water_level',
            name='rob_water_level',
            output='screen',
        ),
        Node(
            package='tb3_state_dt',
            executable='sim_water_level',
            name='sim_water_level',
            output='screen',
        ),
        Node(
            package='tb3_state_dt',
            executable='sim_battery_level',
            name='sim_battery_level',
            output='screen',
        ),
        Node(
            package='tb3_state_dt',
            executable='rob_state',
            name='rob_state',
            output='screen',
        ),
        Node(
            package='tb3_state_dt',
            executable='sim_state',
            name='sim_state',
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(this_dir, 'gazebo_twin.launch.py')
            )
        )
    ])

