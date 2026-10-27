from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition

from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='mst110cr_2')
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    robot_name = LaunchConfiguration('robot_name')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        robot_name_arg,
        use_namespace_arg,
        use_sim_time_arg,

        GroupAction([
            PushRosNamespace(
                condition=IfCondition(use_namespace),
                namespace=robot_name
            ),

            Node(
                package='mst110cr_navigation',
                executable='odom_broadcaster',
                name='odom_broadcaster',
                output="screen",
                parameters=[
                    {'odom_topic': 'odom_pose',
                     'odom_frame': [robot_name, '/odom'],
                     'base_link_frame': [robot_name, '/base_link'],
                     'use_sim_time': use_sim_time,
                    },
                ]
            ),

            Node(
                package='mst110cr_navigation',
                executable='message_converter_gnss',
                name='message_converter_gnss',
                output='screen',
                parameters=[{
                    'input_topic': 'global_pose',
                    'output_topic': 'gnss_pose',
                    'use_sim_time': use_sim_time,
                }],
            ),

            Node(
                package='mst110cr_navigation',
                executable='gnss_ekf_initializer',
                name='gnss_ekf_initializer',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'world_frame': 'map'}],
            ),

            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_global',
                output="screen",
                remappings=[('odometry/filtered', 'odometry/global')],
                parameters=[
                    {
                        'debug': False,
                        'frequency': 10.0,
                        'transform_time_offset': 0.0,
                        'transform_timeout': 0.0,
                        'print_diagnostics': True,
                        'publish_acceleration': True,
                        'publish_tf': True,
                        'two_d_mode': True,
                        'map_frame': 'map',
                        'odom_frame': [robot_name, '/odom'],
                        'base_link_frame': [robot_name, '/base_link'],
                        'world_frame' : 'map',
                        'use_sim_time': use_sim_time,
                        'odom0': 'ekf/odom_pose',
                        # COM3 angular.z is upper-body rate, not base_link rate.
                        # Use lower-body absolute yaw until a corrected rate exists.
                        'odom0_config': [
                            False,  False,  False,
                            False, False, False,
                            True, False, False,
                            False, False, False,
                            False, False, False],
                        'odom0_differential': False,
                        'pose0': 'ekf/gnss_pose',
                        # Fuse antenna x/y only; heading and lever-arm corrections
                        # must be verified before enabling orientation fusion.
                        'pose0_config': [
                            True,  True,  False,
                            False, False, False,
                            False, False, False,
                            False, False, False,
                            False, False, False],
                        'pose0_differential': False,
                        'pose0_relative': False,
                        # Initial GNSS x/y and lower-body yaw; initializer holds all
                        # normal inputs until EKF publishes this initial state.
                        'pose1': 'ekf/initial_pose',
                        'pose1_config': [
                            True, True, False,
                            False, False, True,
                            False, False, False,
                            False, False, False,
                            False, False, False],
                        'pose1_differential': False,
                        'pose1_relative': False,
                        # World-referenced lower-body heading from COM3 /imu.
                        # The initializer gates this input until GNSS initialization.
                        'pose2': 'ekf/lower_heading',
                        'pose2_config': [
                            False, False, False,
                            False, False, True,
                            False, False, False,
                            False, False, False,
                            False, False, False],
                        'pose2_differential': False,
                        'pose2_relative': False,
                        }]
            ),
        ])
    ])
