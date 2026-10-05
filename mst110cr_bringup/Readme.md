# mst110cr_bringup

## remote側

`mst110cr_remote.launch.py`は、`mst110cr_enable_route_switcher_navigation.launch.py`とRVizを起動します。
既定のロボット名・名前空間は`mst110cr_2`です。位置推定と機体TFはvehicle側が配信します。

```bash
source /opt/ros/humble/setup.bash
source /home/opera/mst110cr_ws/install/setup.bash
ros2 launch mst110cr_bringup mst110cr_remote.launch.py use_sim_time:=true
```

bag再生ではvehicle側と同じ`ROS_DOMAIN_ID`を設定し、両方を`use_sim_time:=true`で起動します。
実機では`use_sim_time:=false`（既定値）。GUIなしは`use_rviz:=false`、自動Active化を止める場合は`autostart:=false`。
`map:=...`、`graph:=...`、`params_file:=...`で地図・ルートグラフ・Nav2設定を変更できます。
既定グラフは`mst110cr_navigation/graphs/pwri_dxfield_northeast.geojson`です。

| 用途 | トピック・フレーム |
|---|---|
| TF配信 | `/tf`、`/tf_static` |
| TF構成 | `world → map → mst110cr_2/odom → mst110cr_2/base_link` |
| ルート・global costmap | `map` |
| local costmap | `mst110cr_2/odom` |
| 車体フレーム | `mst110cr_2/base_link` |
| Nav2の速度参照 | `/mst110cr_2/odom_pose` |
| 速度指令 | `/mst110cr_2/cmd_vel_nav → cmd_vel_smoothed → cmd_vel`（すべて同じ名前空間、Twist） |
| 障害物入力 | `/mst110cr_2/scan`（LaserScan、別途配信が必要） |
| ルート切替 | `/mst110cr_2/enable_route_switcher`（Bool、trueでルート使用） |

ルート切替トピックの購読QoSはReliable・Transient Localです。
今回のlocalization2 bagにはscanがないため、位置推定・TF・Nav2起動は確認できますが、障害物入力を含む走行検証には別途scanが必要です。
RVizの2D Pose EstimateはEKFのGNSS初期化をリセットする操作ではありません。

## vehicle側

`mst110cr_vehicle.launch.py`は、GNSS localizer、GNSSで初期化するEKF、odom broadcaster、
`world → map`の静的TF、robot_state_publisherを起動します。
現在、COM3ドライバとNav2のincludeはコメントアウトされています。Nav2はremote側で起動します。

```bash
# 関節名だけ修正した旧bag用。修正済みCOM3入力ではyaw_offset:=0.0を使う。
ros2 launch mst110cr_bringup mst110cr_vehicle.launch.py \
  use_sim_time:=true yaw_offset:=1.5707963267948966
```

GNSS yawオフセットの単位はラジアンです。EKFは最初のGNSS x/y/yawで一度だけ初期化します。
