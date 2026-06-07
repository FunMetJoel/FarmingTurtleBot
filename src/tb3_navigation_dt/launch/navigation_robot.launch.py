import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('tb3_navigation_dt')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    tb3_nav2_params = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'param', 'burger.yaml'
    )

    # Real robot always uses wall clock
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (false for real robot)'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # SLAM — maps the real environment
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'slam_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg_nav, 'config', 'slam_params.yaml'),
        }.items()
    )

    # Nav2
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

    # Water level
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

    # Battery — reads from real /battery_state published by TurtleBot3
    sim_battery_level = Node(
        package='tb3_state_dt',
        executable='sim_battery_level',
        name='sim_battery_level',
        output='screen',
    )

    # Weather context
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
        executable='SafetySupervisorNode',
        name='SafetySupervisorNode',
        output='screen',
    )

    # Dashboard
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
        slam,
        navigation,
        random_humidity_sensor,
        humidity_interpreter,
        humidity_map,
        rob_water_level,
        sim_water_level,
        sim_battery_level,
        weather_adapter,
        twin_safety_supervisor,
        safety_supervisor,
        rosbridge,
        dashboard_bridge,
    ])
