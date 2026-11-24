import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mst110cr_travel_direction'), 'config', 'mst110cr_travel_direction.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='mst110cr_2'),
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='mst110cr_travel_direction',
            executable='travel_direction_adapter',
            name='travel_direction_adapter',
            namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('params_file'), {
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
            output='screen',
        ),
    ])
