import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


use_autostart = True
use_respawn = True


def generate_launch_description():
    mst110cr_navigation_dir = get_package_share_directory('mst110cr_navigation')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    graph_filepath = LaunchConfiguration('graph')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    map_yaml_file = LaunchConfiguration('map', default=os.path.join(mst110cr_navigation_dir, 'map', 'map.yaml'))
    robot_name = LaunchConfiguration('robot_name')
    # use_navigation_xy_goal_tolerance = LaunchConfiguration('navigation_xy_goal_tolerance')
    # use_navigation_yaw_goal_tolerance = LaunchConfiguration('navigation_yaw_goal_tolerance')

    # use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value='true')
    # use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    # use_navigation_xy_goal_tolerance_arg = DeclareLaunchArgument('navigation_xy_goal_tolerance', default_value='1.0')
    # use_navigation_yaw_goal_tolerance_arg = DeclareLaunchArgument('navigation_yaw_goal_tolerance', default_value='0.15')

    lifecycle_nodes_localization = ['map_server']

    lifecycle_nodes_navigation = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'route_server',  # new
        'behavior_server',
        'velocity_smoother',
        'bt_navigator',
        'waypoint_follower',
        'docking_server',  # new
    ]

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    remappings = [('tf', '/tf'), ('tf_static', '/tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': os.path.join(mst110cr_navigation_dir, 'params', 'enable_route_compute_and_track_route_graph_2.xml'),
        'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml': os.path.join(mst110cr_navigation_dir, 'params', 'enable_route_compute_and_track_route_graph_2.xml'),
        'autostart': autostart,
        'use_sim_time': use_sim_time,
        # map yaml
        'map_server.ros__parameters.yaml_filename': map_yaml_file,

        # bt_navigator
        'bt_navigator.ros__parameters.robot_base_frame': [robot_name, '/base_link'],
        'bt_navigator.ros__parameters.odom_topic': ['/', robot_name, '/odom_pose'],

        # behavior_server
        'behavior_server.ros__parameters.robot_base_frame': [robot_name, '/base_link'],
        'behavior_server.ros__parameters.local_frame': [robot_name, '/odom'],

        # controller_server
        'controller_server.ros__parameters.odom_topic': ['/', robot_name, '/odom_pose'],
        # 'controller_server.ros__parameters.general_goal_checker.xy_goal_tolerance': use_navigation_xy_goal_tolerance,
        # 'controller_server.ros__parameters.general_goal_checker.yaw_goal_tolerance': use_navigation_yaw_goal_tolerance,

        # velocity_smoother
        'velocity_smoother.ros__parameters.odom_topic': ['/', robot_name, '/odom_pose'],

        # costmaps
        'local_costmap.local_costmap.ros__parameters.global_frame': [robot_name, '/odom'],
        'local_costmap.local_costmap.ros__parameters.robot_base_frame': [robot_name, '/base_link'],
        'global_costmap.global_costmap.ros__parameters.robot_base_frame': [robot_name, '/base_link'],

        # docking server
        'docking_server.ros__parameters.base_frame': [robot_name, '/base_link'],
        'docking_server.ros__parameters.fixed_frame': [robot_name, '/odom'],

        # collision_monitor
        'collision_monitor.ros__parameters.base_frame_id': [robot_name, '/base_link'],
        'collision_monitor.ros__parameters.odom_frame_id': [robot_name, '/odom'],

        # loopback_simulator
        'loopback_simulator.ros__parameters.base_frame_id': [robot_name, '/base_link'],
        'loopback_simulator.ros__parameters.odom_frame_id': [robot_name, '/odom'],

        # route_server
        'route_server.ros__parameters.base_frame': [robot_name, '/base_link'],
        'route_server.ros__parameters.route_frame': ['map']
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True
        ),
        allow_substs=True
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value=LaunchConfiguration('robot_name'), description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(mst110cr_navigation_dir, 'params', 'mst110cr_enable_route_switcher_navigation.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_graph_file_cmd = DeclareLaunchArgument(
        'graph',
        default_value=os.path.join(mst110cr_navigation_dir, 'graphs', 'pwri_dxfield_northeast.geojson'), description='Path to the graph file to load'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Use composed bringup if True',
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='the name of container that nodes will load in if use composition',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name', default_value='mst110cr_2'
    )

    load_localization_nodes = GroupAction([
        PushRosNamespace(
            # condition=IfCondition(True),
            namespace=namespace
        ),

        # #########################
        # # Localization packages #
        # #########################
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params, {'use_sim_time': use_sim_time, 'enable_stamped_cmd_vel': False}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'autostart': autostart,
                'use_sim_time': use_sim_time,
                'node_names': lifecycle_nodes_localization
            }]
        ),
        Node(
            package='rclcpp_components',
            executable='component_container_isolated',
            name=container_name,
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen',
        )
    ])


    load_navigation_composable_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_controller',
                        plugin='nav2_controller::ControllerServer',
                        name='controller_server',
                        parameters=[configured_params, {'use_sim_time': use_sim_time, 'enable_stamped_cmd_vel': False}],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                        namespace=namespace,
                    ),
                    ComposableNode(
                        package='nav2_smoother',
                        plugin='nav2_smoother::SmootherServer',
                        name='smoother_server',
                        parameters=[configured_params, {'use_sim_time': use_sim_time, 'enable_stamped_cmd_vel': False}],
                        remappings=remappings,
                        namespace=namespace,
                    ),
                    ComposableNode(
                        package='nav2_planner',
                        plugin='nav2_planner::PlannerServer',
                        name='planner_server',
                        parameters=[configured_params, {'use_sim_time': use_sim_time, 'enable_stamped_cmd_vel': False}],
                        remappings=remappings,
                        namespace=namespace,
                    ),
                    ComposableNode(
                        package='nav2_route',
                        plugin='nav2_route::RouteServer',
                        name='route_server',
                        parameters=[configured_params, {'graph_filepath': graph_filepath, 'use_sim_time': use_sim_time}],
                        remappings=remappings,
                        namespace=namespace,
                    ),
                    ComposableNode(
                        package='nav2_behaviors',
                        plugin='behavior_server::BehaviorServer',
                        name='behavior_server',
                        parameters=[configured_params, {'use_sim_time': use_sim_time, 'enable_stamped_cmd_vel': False}],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                        namespace=namespace,
                    ),
                    ComposableNode(
                        package='nav2_bt_navigator',
                        plugin='nav2_bt_navigator::BtNavigator',
                        name='bt_navigator',
                        parameters=[configured_params, {'use_sim_time': use_sim_time, 'enable_stamped_cmd_vel': False}],
                        remappings=remappings,
                        namespace=namespace,
                    ),
                    ComposableNode(
                        package='nav2_waypoint_follower',
                        plugin='nav2_waypoint_follower::WaypointFollower',
                        name='waypoint_follower',
                        parameters=[configured_params, {'use_sim_time': use_sim_time, 'enable_stamped_cmd_vel': False}],
                        remappings=remappings,
                        namespace=namespace,
                    ),
                    ComposableNode(
                        package='nav2_velocity_smoother',
                        plugin='nav2_velocity_smoother::VelocitySmoother',
                        name='velocity_smoother',
                        parameters=[configured_params, {'use_sim_time': use_sim_time, 'enable_stamped_cmd_vel': False}],
                        remappings=remappings + [
                            ('cmd_vel', 'cmd_vel_nav'),
                            ('cmd_vel_smoothed', 'cmd_vel'),
                        ],
                        namespace=namespace,
                    ),
                    ComposableNode(
                        package='opennav_docking',
                        plugin='opennav_docking::DockingServer',
                        name='docking_server',
                        parameters=[configured_params, {'use_sim_time': use_sim_time, 'enable_stamped_cmd_vel': False}],
                        remappings=remappings,
                        namespace=namespace,
                    ),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_navigation',
                        parameters=[
                            {'autostart': autostart,
                             'use_sim_time': use_sim_time,
                             'node_names': lifecycle_nodes_navigation}
                        ],
                        namespace=namespace,
                    ),
                ],
            ),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_graph_file_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_localization_nodes)
    ld.add_action(load_navigation_composable_nodes)

    return ld
