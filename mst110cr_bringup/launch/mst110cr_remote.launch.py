"""Remote standard navigation and RViz; vehicle provides localization/TF."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    nav = get_package_share_directory('mst110cr_navigation')
    robot = LaunchConfiguration('robot_name')
    sim = LaunchConfiguration('use_sim_time')
    arguments = {
        'robot_name': 'mst110cr_2',
        'use_sim_time': 'false',
        'autostart': 'true',
        'use_rviz': 'false',
        'map': os.path.join(nav, 'map', 'map.yaml'),
        'params_file': os.path.join(nav, 'params', 'navigation_parameters.yaml'),
        'rviz_config': "/home/opera/mst110cr_ws/src/mst110cr_ros2/mst110cr_navigation/rviz2/mst110cr_navigation.rviz",
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in arguments.items()],
        # 通常ナビゲーション。切り替える場合はこの IncludeLaunchDescription 全体を
        # コメントアウトし、直下の route switcher 用ブロックを有効にする。
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(
        #         nav, 'launch', 'mst110cr_navigation.launch.py')),
        #     launch_arguments={
        #         'robot_name': robot,
        #         'use_namespace': 'true',
        #         'use_sim_time': sim,
        #         'autostart': LaunchConfiguration('autostart'),
        #         'map': LaunchConfiguration('map'),
        #         'params_file': LaunchConfiguration('params_file'),
        #     }.items(),
        # ),
        # Route switcher 用（上の通常ナビゲーションとどちらか一方だけ有効にする）。
        # 親の params_file は通常用なので、ここでは route switcher 用 YAML を明示する。
        # graph は子 launch のデフォルト（261119-kyoto_1_itsuka_mst110cr_zone.geojson）を使用。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                nav, 'launch', 'mst110cr_enable_route_switcher_navigation.launch.py')),
            launch_arguments={
                'robot_name': robot,
                'namespace': robot,
                'use_sim_time': sim,
                'autostart': LaunchConfiguration('autostart'),
                'map': LaunchConfiguration('map'),
                'params_file': os.path.join(
                    nav, 'params', 'mst110cr_enable_route_switcher_navigation.yaml'),
            }.items(),
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            package='rviz2', executable='rviz2', name='rviz', namespace=robot,
            arguments=['--display-config', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': ParameterValue(sim, value_type=bool)}],
        ),
    ])
