import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap
from launch_ros.actions import Node


def generate_launch_description():
    dashboard_dir = get_package_share_directory('tb3_dashboard')
    field_dir = get_package_share_directory('tb3_field_dt')
    navigation_dir = get_package_share_directory('tb3_navigation_dt')
    state_dir = get_package_share_directory('tb3_state_dt')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(dashboard_dir, 'launch', 'main.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_rviz': 'False'
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(field_dir, 'launch', 'main.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_rviz': 'False'
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(navigation_dir, 'launch', 'main.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_rviz': 'False'
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(state_dir, 'launch', 'main.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_rviz': 'False'
            }.items()
        ),

        use_sim_time_arg
    ])




