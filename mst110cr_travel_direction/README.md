# mst110cr_travel_direction

上部旋回角 `rotate_joint` に応じて、速度指令の前後を補正するROS 2 Pythonパッケージ。
Nav2後段に置く独立ノードで、pluginlibプラグインやLifecycleノードではありません。

```text
Nav2 → cmd_vel_nav → velocity_smoother → cmd_vel_smoothed
                                              ↓
joint_states → travel_direction_adapter → cmd_vel → COM3ドライバ
```

## 現在の導入範囲

`mst110cr_remote.launch.py` が現在使用している
`mst110cr_enable_route_switcher_navigation.launch.py` に組み込んでいます。
この子launchが本ノードを同じ名前空間・`use_sim_time` で起動し、
smootherの出力を `cmd_vel_smoothed` のまま本ノードへ渡します。
COM3ドライバの入力は従来どおり `cmd_vel` です。
このroute switcher用launchでは補正を既定で有効にします。
`enable_travel_direction_correction:=false` を指定すると、そのまま転送します。
この起動引数は設定ファイルの `enable_correction` より優先されます。
単体起動では設定ファイルに従い、既定は補正無効です。

補正ノード用の設定は `travel_direction_params_file:=/path/to/config.yaml` で指定できます。
Nav2用の `params_file` とは別です。remote起動時に単体launchを重ねて起動しないでください。
通常の `mst110cr_navigation.launch.py` と `mst110cr_route_navigation.launch.py` は
今回の接続変更の対象外です。remoteの起動先を切り替える場合は接続構成も確認してください。

```bash
colcon build --packages-select mst110cr_travel_direction mst110cr_navigation mst110cr_bringup --symlink-install
source install/setup.bash
ros2 launch mst110cr_bringup mst110cr_remote.launch.py
```

## ビルドと単体起動

```bash
colcon build --packages-select mst110cr_travel_direction --symlink-install
source install/setup.bash
ros2 launch mst110cr_travel_direction mst110cr_travel_direction.launch.py
```

`namespace`（既定値 `mst110cr_2`）、`params_file`、`use_sim_time` を指定できます。
実機と分離した試験では、例えば `namespace:=travel_direction_test` を指定します。
角度とTFの時刻系に合わせて `use_sim_time` を設定してください。

## 動作

- 既定の `enable_correction: false` では、角度の有無にかかわらずTwistをそのまま転送します。
- 入力1件につき出力1件。指令の周期再送はしません。入力が止まると出力も止まり、
  既存COM3ドライバの0.5秒タイムアウトによるゼロ指令を維持します。
- 補正有効時は角度を±πに正規化し、絶対角度が `reverse_threshold_deg` を超えると
  `linear.x` を反転します。`invert_angular_z: true` の場合だけ `angular.z` も反転します。
- 閾値±`stop_band_deg` の範囲（端点を含む）では入力に対してゼロTwistを出します。
  既定値0では閾値ちょうどだけが停止対象です。ヒステリシスや停止確認の待機処理はありません。
- 補正有効時、角度未受信・非有限角度・position欠損・古い角度・未来時刻の場合は
  入力に対してゼロTwistを出します。受信からの経過時間とJointStateの送信時刻を
  両方確認します。他の関節だけのメッセージは角度の鮮度を更新しません。
- 補正有効時は非有限値を含むTwistもゼロにします。
- 直接 `cmd_vel` に送った指令は補正対象外です。Nav2との同時送信は調停しません。
- 入出力トピックがリマップ後に同名になる設定は、自己ループ防止のため拒否します。

`config/mst110cr_travel_direction.yaml` に設定をまとめています。パラメーターは起動時設定で、
実行中の変更はできません。90度の閾値と角速度を反転しない設定は仮の仕様です。
機体側の前後切替条件と旋回方向を確認してから補正を有効にしてください。

診断用 `~/state`（std_msgs/String）を5 Hzで配信します：
`passthrough` / `waiting_for_angle` / `stale_angle` / `boundary_stop` / `forward` / `reverse`。
このタイマーは走行指令を送信しません。

## テスト

```bash
colcon test --packages-select mst110cr_travel_direction
colcon test-result --verbose
```
