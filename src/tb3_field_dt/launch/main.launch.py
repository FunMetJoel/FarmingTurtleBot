from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    return LaunchDescription([
        use_sim_time_arg,
        Node(
            package='tb3_field_dt',
            executable='fieldHumiditySensor',
            name='fieldHumiditySensor',
        ),
        Node(
            package='tb3_field_dt',
            executable='humidityMap',
            name='humidityMap',
        ),
        Node(
            package='tb3_field_dt',
            executable='humiditySensorInterpreter',
            name='humiditySensorInterpreter',
            output='screen'
        ),
    ])

