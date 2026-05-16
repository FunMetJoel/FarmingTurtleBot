import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    pkg_share = get_package_share_directory('tb3_field_dt')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'HumidityMapPlotter.rviz')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    return LaunchDescription([
        use_sim_time_arg,
        Node(
            package='tb3_field_dt',
            executable='randomHumiditySensor',
            name='randomHumiditySensor',
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
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path, 'use_sim_time', LaunchConfiguration('use_sim_time')],
            output='screen'
        )
    ])


# 'randomHumiditySensor = tb3_field_dt.RandomHumiditySensorNode:main',
# 'humiditySensorInterpreter = tb3_field_dt.HumiditySensorInterpreterNode:main',
# 'humidityMap = tb3_field_dt.HumidityMapNode:main'
        