import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import AppendEnvironmentVariable


def generate_launch_description():
        
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')


    world_path = PathJoinSubstitution([
        FindPackageShare('tb3_state_dt'),
        'worlds',
        'sim_world.world'
    ])

    set_env_vars_resources  = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'models'))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': ['-r -v2 ', world_path],
            'use_sim_time': use_sim_time,
        }.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')),
        launch_arguments={'-topic': '/sim/robot_description', '-name': 'turtlebot3', 'x_pose': '0', 'y_pose': '0', 'use_sim_time': use_sim_time}.items()
    )

    
    twin_simulation_group = GroupAction([
        PushRosNamespace('sim'),
        SetEnvironmentVariable(name='GZ_PARTITION', value='digital_twin_partition'),
        gazebo,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
        set_env_vars_resources
    ])

    return LaunchDescription([
        twin_simulation_group
    ])
