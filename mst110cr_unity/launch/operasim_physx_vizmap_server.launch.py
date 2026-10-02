import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition


use_autostart = True
use_respawn = True


def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='mst110cr')
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    # use_navigation_xy_goal_tolerance_arg = DeclareLaunchArgument('navigation_xy_goal_tolerance', default_value='1.0')
    # use_navigation_yaw_goal_tolerance_arg = DeclareLaunchArgument('navigation_yaw_goal_tolerance', default_value='0.15')

    robot_name = LaunchConfiguration('robot_name')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    lifecycle_nodes_localization = ['vizmap_server']

    def launch_setup(context, *args, **kwargs):
        # use_sim_time_str = use_sim_time.perform(context).strip().lower()
        # use_sim_bool = use_sim_time_str in ('true', '1', 'yes', 'y', 'on')
        # nav_params_file = navigation_parameters_sim_yaml_file if use_sim_bool else navigation_parameters_yaml_file

        # param_substitutions = {
        #     # map yaml
        #     'yaml_filename': map_yaml_file,
        # }

        # configured_params = RewrittenYaml(
        #     source_file=nav_params_file,
        #     root_key=robot_name,
        #     param_rewrites=param_substitutions,
        #     convert_types=True
        # )

        return [
            GroupAction([
                PushRosNamespace(
                    condition=IfCondition(use_namespace),
                    namespace=robot_name
                ),

                # Node(
                #     condition=IfCondition('true'),
                #     name='nav2_container',
                #     package='rclcpp_components',
                #     executable='component_container_isolated',
                #     parameters=[
                #         configured_params,
                #         {
                #             'autostart': use_autostart,
                #         }
                #     ],
                #     output='screen'
                # ),

                #########################
                # Localization packages #
                #########################
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='vizmap_server',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[{
                        'yaml_filename': '/home/opera/mst110cr_ws/src/mst110cr_ros2/mst110cr_unity/map/operasim_physx_vizmap.yaml',
                        'topic_name': 'vizmap',
                        'frame_id': 'map',
                        'introspection_mode': 'disabled',
                        'use_sim_time': use_sim_time
                    }],
                ),

                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_vizmap',
                    output='screen',
                    parameters=[{
                        'autostart': use_autostart,
                        'node_names': lifecycle_nodes_localization,
                        'use_sim_time': use_sim_time
                    }]
                ),
                
            ])
        ]

    return LaunchDescription([
        robot_name_arg,
        use_namespace_arg,
        use_sim_time_arg, 

        OpaqueFunction(function=launch_setup),
    ])