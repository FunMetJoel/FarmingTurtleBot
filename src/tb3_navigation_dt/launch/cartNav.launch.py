import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap

def generate_launch_description():
    cartographer_dir = get_package_share_directory('turtlebot3_cartographer')
    navigation2_dir = get_package_share_directory('turtlebot3_navigation2')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(cartographer_dir, 'launch', 'cartographer.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_rviz': 'False'
            }.items()
        ),

        GroupAction(
            actions=[
                SetRemap(src='/cmd_vel', dst='/cmd_vel_raw'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(navigation2_dir, 'launch', 'navigation2.launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'slam': 'False'
                    }.items()
                ),
            ]
        ),

        use_sim_time_arg
    ])




