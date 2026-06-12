import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    pkg_nav = get_package_share_directory('tb3_navigation_dt')
    pkg_world = get_package_share_directory('my_tb3_world')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    tb3_nav2_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_world, 'launch', 'new_world.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'slam_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg_nav, 'config', 'slam_params.yaml'),
        }.items()
    )

    navigation = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/cmd_vel_raw'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': tb3_nav2_params,
                }.items()
            ),
        ]
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

    # Water level (robot sensor + digital twin)
    rob_water_level = Node(
        package='tb3_state_dt',
        executable='rob_water_level',
        name='rob_water_level',
        output='screen',
    )
    sim_water_level = Node(
        package='tb3_state_dt',
        executable='sim_water_level',
        name='sim_water_level',
        output='screen',
    )

    # Weather context for digital twin
    weather_adapter = Node(
        package='tb3_weather_dt',
        executable='weather_adapter',
        name='weather_adapter',
        output='screen',
    )
    twin_safety_supervisor = Node(
        package='tb3_weather_dt',
        executable='twin_safety_supervisor',
        name='twin_safety_supervisor',
        output='screen',
    )

    # Safety supervisor
    safety_supervisor = Node(
        package='tb3_navigation_dt',
        executable='safetySupervisorNode',
        name='SafetySupervisorNode',
        output='screen',
    )

    # Coverage: frontier exploration + zigzag humidity sweep
    coverage_orchestrator = Node(
        package='tb3_navigation_dt',
        executable='coverageOrchestrator',
        name='coverageOrchestrator',
        output='screen',
    )

    # Dashboard
    dashboard_server = Node(
        package='tb3_dashboard',
        executable='dashboard_server',
        name='dashboard_server',
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
        rob_water_level,
        sim_water_level,
        weather_adapter,
        twin_safety_supervisor,
        safety_supervisor,
        coverage_orchestrator,
        dashboard_server,
    ])