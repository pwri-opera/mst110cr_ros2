# mst110cr_navigation

## launch

### ekf_localization.launch.py

以下を起動
* mst110cr_navigation/odom_broadcaster
* mst110cr_navigation/poseStamped2Odometry
* robot_localization/ekf_node


### mst110cr_navigation.launch.py

* rclcpp_components/component_container_isolated

* nav2_map_server/map_server
* nav2_lifecycle_manager/lifecycle_manager_localization

Navigation2用のサーバ群を起動
* nav2_controller/controller_server
* nav2_smoother/smoother_server
* nav2_planner/planner_server
* nav2_behaviors/behavior_server
* nav2_bt_navigator/bt_navigator
* nav2_waypoint_follower/waypoint_follower
* nav2_velocity_smoother/velocity_smoother
* nav2_lifecycle_manager/lifecycle_manager
