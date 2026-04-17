import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch.conditions import IfCondition

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mst110cr_unity_dir = get_package_share_directory("mst110cr_unity")
    mst110cr_navigation_dir = get_package_share_directory("mst110cr_navigation")

    operasim_physx_vizmap_file_path = os.path.join(
        mst110cr_unity_dir, "launch", "operasim_physx_vizmap_server.launch.py"
    )
    ekf_localization_launch_file_path = os.path.join(
        mst110cr_navigation_dir, "launch", "ekf_localization.launch.py"
    )
    mst110cr_navigation_launch_file_path = os.path.join(
        mst110cr_navigation_dir, "launch", "mst110cr_navigation.launch.py"
    )
    mst110cr_route_navigation_launch_file_path = os.path.join(
        mst110cr_navigation_dir, "launch", "mst110cr_route_navigation.launch.py"
    )
    mst110cr_standby_rviz_file_path = os.path.join(
        mst110cr_navigation_dir, "rviz2", "mst110cr_navigation_sim.rviz"
    )

    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='mst110cr')
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    use_navigation_xy_goal_tolerance_arg = DeclareLaunchArgument('navigation_xy_goal_tolerance', default_value='1.0')
    use_navigation_yaw_goal_tolerance_arg = DeclareLaunchArgument('navigation_yaw_goal_tolerance', default_value='0.15')

    robot_name = LaunchConfiguration('robot_name')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_navigation_xy_goal_tolerance = LaunchConfiguration('navigation_xy_goal_tolerance')
    use_navigation_yaw_goal_tolerance = LaunchConfiguration('navigation_yaw_goal_tolerance')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('mst110cr_description'),
        'urdf',
        'mst110cr.xacro'
    ])

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ',
                xacro_file, ' ',
                'prefix:=', robot_name
            ]),
            value_type=str
        )
    }

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output="screen",
        remappings=[
            ('odometry/filtered', 'odometry/global'),
            # ('odom0', 'odom_pose'),
            # ('odom1', 'gnss_odom'),
        ],
        parameters=[{
            'debug': False,
            'frequency': 20.0,
            'transform_time_offset': 0.1,
            'transform_timeout': 0.0,
            'print_diagnostics': True,
            'publish_acceleration': True,
            'publish_tf': True,
            'two_d_mode': True,
            'map_frame': 'map',
            'odom_frame': [robot_name, '/odom'],
            'base_link_frame': [robot_name, '/base_link'],
            'world_frame': 'map',
            'use_sim_time': use_sim_time,

            'odom0': 'mst110cr/odom_pose',
            'odom0_config': [
                True,  True,  False,
                False, False, True,
                False, False, False,
                False, False, False,
                False, False, False],
            'odom0_differential': True,

            'odom1': 'mst110cr/global_pose',
            'odom1_config': [
                True,  True,  True,
                False, False, True,
                False, False, False,
                False, False, False,
                False, False, False],
            'odom1_differential': False,
        }]
    )
    

    return LaunchDescription([
        robot_name_arg,
        use_namespace_arg,
        use_sim_time_arg,
        use_navigation_xy_goal_tolerance_arg,
        use_navigation_yaw_goal_tolerance_arg,


        GroupAction([
            PushRosNamespace(
                condition=IfCondition(use_namespace),
                namespace=robot_name
            ),

            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='world_to_map',
                arguments=[
                    '--x','0','--y','0','--z','0',
                    '--roll','0','--pitch','0','--yaw','0',
                    '--frame-id','world',
                    '--child-frame-id','map'
                ],
                parameters=[{'use_sim_time': use_sim_time}],
            ),

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[robot_description, {'use_sim_time': use_sim_time}],
            ),

            # Node(
            #     package="rviz2",
            #     executable="rviz2",
            #     name="rviz",
            #     arguments=["--display-config", mst110cr_standby_rviz_file_path],
            #     parameters=[{'use_sim_time': use_sim_time}],
            # ),
        ]),


        ekf_node,

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(mst110cr_navigation_launch_file_path),
        #     launch_arguments={
        #         'robot_name': robot_name,
        #         'use_namespace': 'true',
        #         'use_sim_time': use_sim_time,
        #         'navigation_xy_goal_tolerance': use_navigation_xy_goal_tolerance,
        #         'navigation_yaw_goal_tolerance': use_navigation_yaw_goal_tolerance
        #     }.items(),
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mst110cr_route_navigation_launch_file_path),
            launch_arguments={
                'robot_name': robot_name,
                'use_namespace': 'true',
                'use_sim_time': use_sim_time,
                'navigation_xy_goal_tolerance': use_navigation_xy_goal_tolerance,
                'navigation_yaw_goal_tolerance': use_navigation_yaw_goal_tolerance
            }.items(),
        ),


        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/mst110cr/livox_fr'),
                        ('scan', '/mst110cr/livox_fr_scan')],
            parameters=[{
                'target_frame': 'mst110cr/livox_fr',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 0.05,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 20.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan_livox_fr'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/mst110cr/livox_fl'),
                        ('scan', '/mst110cr/livox_fl_scan')],
            parameters=[{
                'target_frame': 'mst110cr/livox_fl',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 0.05,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 20.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan_livox_fl'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/mst110cr/livox_rr'),
                        ('scan', '/mst110cr/livox_rr_scan')],
            parameters=[{
                'target_frame': 'mst110cr/livox_rr',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 0.05,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 20.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan_livox_rr'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/mst110cr/livox_rl'),
                        ('scan', '/mst110cr/livox_rl_scan')],
            parameters=[{
                'target_frame': 'mst110cr/livox_rl',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 0.05,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 20.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan_livox_rl'
        ),
    ])