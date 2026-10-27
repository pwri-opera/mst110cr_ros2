# mst110cr_navigation

## launch

### ekf_localization.launch.py

以下を起動
* mst110cr_navigation/odom_broadcaster
* mst110cr_navigation/message_converter_gnss
* mst110cr_navigation/gnss_ekf_initializer
* robot_localization/ekf_node

車両側の `mst110cr_vehicle.launch.py` から起動する。
`mst110cr_remote.launch.py` は車両側のEKF/TFを利用し、EKFを重複起動しない。

GNSSのx/yと下部走行体の `imu.orientation` のyawで一度初期化する。
両入力が有効かつ0.5秒以内（未来は0.1秒以内）になるまで待ち、
それぞれをmap座標へ変換する。待機中は最新のGNSS位置を保持し、
初期yawの分散にも下部IMUの値を使う。上部姿勢や `yaw_offset` は初期yawに使わない。
初期化後も同じ下部走行体の `imu.orientation` を
`ekf/lower_heading` (PoseWithCovarianceStamped) に変換し、`pose2` のyawのみを
絶対方位として継続融合する。COM3のIMUは `header.frame_id=world` が方位の基準を
表すため、通常のIMU取り付けフレームとして `imu0` に直接入力しない。
初期化前のheadingは転送せず、無効姿勢（共分散先頭が-1）、非正のyaw分散、
古いメッセージ（0.5秒超）、0.1秒超の未来時刻、重複・逆順時刻を除外する。
この鮮度判定はメッセージ時刻に対するもので、TCP client内で古い受信値に新しい
時刻を付けるケースの検出には受信側の品質・鮮度管理が別途必要。

COM3側の `/imu` yaw共分散を有効にしたバージョンが必要（開始値0.01 rad²）。
古いbagの `orientation_covariance[0]=-1` は自動で有効扱いに変更しない。
`odom0` は前後速度のみを融合する。未補正の上部角速度は下部yawと矛盾するため
融合しない。有効な下部headingがない場合は初期化を待機する。
初期化後にheadingが途絶えた場合、角速度の実測入力はない。


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
