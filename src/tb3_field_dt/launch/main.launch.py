from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    return LaunchDescription([
        use_sim_time_arg,
        Node(
            package='tb3_field_dt',
            executable='fieldHumiditySensor',
            name='fieldHumiditySensor',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='tb3_field_dt',
            executable='humidityMap',
            name='humidityMap',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='tb3_field_dt',
            executable='humiditySensorInterpreter',
            name='humiditySensorInterpreter',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])

