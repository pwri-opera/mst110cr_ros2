import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import xacro

robot_name="mst110cr_2"
use_namespace=True

def generate_launch_description():

    use_navigation_xy_goal_tolerance_arg = DeclareLaunchArgument('navigation_xy_goal_tolerance', default_value='1.0')
    use_navigation_yaw_goal_tolerance_arg = DeclareLaunchArgument('navigation_yaw_goal_tolerance', default_value='0.15')    
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    
    mst110cr_com3_ros_dir = get_package_share_directory("mst110cr_com3ros")
    gnss_localizer_ros2 = get_package_share_directory("gnss_localizer_ros2")
    mst110cr_navigation_dir=get_package_share_directory("mst110cr_navigation")
    mst110cr_description_dir=get_package_share_directory("mst110cr_description")

    mst110cr_com3_ros_launch_file = os.path.join(mst110cr_com3_ros_dir, 'launch', 'mst110cr_com3_ros.launch.py')
    gnss_localizer_ros2_launch_file_path=os.path.join(gnss_localizer_ros2, "launch","gnss_localizer_ros2.launch.py")
    ekf_localization_launch_file_path=os.path.join(mst110cr_navigation_dir,"launch","ekf_localization.launch.py")
    mst110cr_navigation_launch_file_path=os.path.join(mst110cr_navigation_dir,"launch","mst110cr_navigation.launch.py")

    xacro_model = os.path.join(mst110cr_description_dir, "urdf", "mst110cr.xacro")
    doc = xacro.parse(open(xacro_model))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    use_navigation_xy_goal_tolerance = LaunchConfiguration('navigation_xy_goal_tolerance')
    use_navigation_yaw_goal_tolerance = LaunchConfiguration('navigation_yaw_goal_tolerance')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        use_navigation_xy_goal_tolerance_arg,
        use_navigation_yaw_goal_tolerance_arg,
        use_sim_time_arg,
        DeclareLaunchArgument("yaw_offset", default_value="0.0"),

        # IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource(mst110cr_com3_ros_launch_file),
        # ),

        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gnss_localizer_ros2_launch_file_path),
                launch_arguments={'use_sim_time': use_sim_time,
                                  'yaw_offset': LaunchConfiguration('yaw_offset'),
                                  'plane': '6'}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_localization_launch_file_path),
            launch_arguments={
                'robot_name': robot_name,
                'use_namespace': 'true',
                'use_sim_time': use_sim_time,
            }.items(),
        ),

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

        GroupAction([
            PushRosNamespace(
                condition=IfCondition(str(use_namespace)),
                namespace=robot_name),

            DeclareLaunchArgument('robot_name', default_value=robot_name),

            # 土研（９系）
            # Node(
            #     package='tf2_ros',
            #     executable='static_transform_publisher',
            #     name='world_to_map',
            #     arguments=[
            #         '--x','21395.178', 
            #         '--y','14034.450', 
            #         '--z','28.552', 
            #         '--roll','0', 
            #         '--pitch','0', 
            #         '--yaw','0', 
            #         '--frame-id', 'world',
            #         '--child-frame-id', 'map'
            #     ],
            #     parameters=[
            #         params,
            #         {
            #             'use_sim_time': use_sim_time
            #         }
            #     ]
            # ),
            
            # 京都(6系)
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='world_to_map',
                arguments=[
                    '--x','-19317.145', 
                    '--y','-138648.38', 
                    '--z','52.157', 
                    '--roll','0', 
                    '--pitch','0', 
                    '--yaw','0', 
                    '--frame-id', 'world',
                    '--child-frame-id', 'map'
                ],
                parameters=[
                    params,
                    {
                        'use_sim_time': use_sim_time
                    }
                ]
            ),

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[
                    params,
                    {
                    'use_sim_time': use_sim_time
                    }
                ]
            ),
        ]),
    ])
