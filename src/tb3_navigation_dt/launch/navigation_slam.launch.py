import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('tb3_navigation_dt')
    pkg_world = get_package_share_directory('my_tb3_world')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    tb3_nav2_params = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'param', 'burger.yaml'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Gazebo world + robot
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_world, 'launch', 'new_world.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # SLAM Toolbox in online async mapping mode
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'slam_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg_nav, 'config', 'slam_params.yaml'),
        }.items()
    )

    # Nav2 navigation stack (planners, controllers, behaviours)
    # Use turtlebot3_navigation2's burger.yaml — it's the correct Jazzy-compatible format
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': tb3_nav2_params,
        }.items()
    )

    # Humidity pipeline
    random_humidity_sensor = Node(
        package='tb3_field_dt',
        executable='randomHumiditySensor',
        name='randomHumiditySensor',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    humidity_interpreter = Node(
        package='tb3_field_dt',
        executable='humiditySensorInterpreter',
        name='humiditySensorInterpreter',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    humidity_map = Node(
        package='tb3_field_dt',
        executable='humidityMap',
        name='humidityMap',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Coverage orchestrator — waits for Nav2 internally before doing anything
    coverage = Node(
        package='tb3_navigation_dt',
        executable='coverageOrchestrator',
        name='coverageOrchestrator',
        output='screen',
    )

    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'port': 9090}],
        output='screen',
    )

    dashboard_bridge = Node(
        package='tb3_dashboard',
        executable='dashboard_bridge',
        name='dashboard_bridge',
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        gazebo,
        slam,
        navigation,
        random_humidity_sensor,
        humidity_interpreter,
        humidity_map,
        coverage,
        rosbridge,
        dashboard_bridge,
    ])
