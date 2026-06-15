import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def include_launch(package, filename, use_sim_time):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package),
                'launch',
                filename,
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    field_stack = GroupAction([
        SetRemap(src='/irrigate', dst='/irrigating'),
        include_launch('tb3_field_dt', 'main.launch.py', use_sim_time),
    ])

    mapping_drive = ExecuteProcess(
        cmd=[
            'timeout', '--signal=INT', '45s',
            'ros2', 'run', 'tb3_navigation_dt', 'backAndForth',
            '--ros-args', '-p', 'use_sim_time:=true',
        ],
        output='screen',
    )

    irrigation_follower = Node(
        package='tb3_field_dt',
        executable='Nav2IrrigationRouteFollower',
        name='Nav2IrrigationRouteFollower',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    irrigation_planner = Node(
        package='tb3_field_dt',
        executable='irrigationRoutePlanner',
        name='irrigationRoutePlanner',
        parameters=[{
            'use_sim_time': use_sim_time,
            'moisture_threshold': 80.0,
            'minimum_dry_nodes': 3,
            'extra_time_factor': 1.0,
        }],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            os.path.join(
                get_package_share_directory('tb3_field_dt'),
                'rviz',
                'MinimalHumidityDemo.rviz',
            ),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        additional_env={
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'QT_OPENGL': 'software',
            'QT_X11_NO_MITSHM': '1',
            'OGRE_RTT_MODE': 'Copy',
            'XDG_CONFIG_HOME': '/tmp/rviz_demo_config',
            'XDG_CACHE_HOME': '/tmp/rviz_demo_cache',
        },
        output='screen',
    )

    start_irrigation = RegisterEventHandler(
        OnProcessExit(
            target_action=mapping_drive,
            on_exit=[
                irrigation_follower,
                TimerAction(period=2.0, actions=[irrigation_planner]),
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        include_launch('my_tb3_world', 'new_world.launch.py', use_sim_time),
        include_launch('tb3_navigation_dt', 'main.launch.py', use_sim_time),
        field_stack,
        rviz,
        TimerAction(period=15.0, actions=[mapping_drive]),
        start_irrigation,
    ])
