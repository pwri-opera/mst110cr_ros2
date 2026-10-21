# bt_control_nodes

MST110CRのBehavior Tree（BT）で使用するカスタムノードを提供するROS 2パッケージです。ログ出力、ゴールまでの距離に応じた子ノードの実行、クローラの前後方向に合わせた上部旋回を扱います。

> このREADMEは現在のソースコードに基づく草案です。各ノードはBehaviorTree.CPPの共有ライブラリとして読み込みます。単独で `ros2 run` する実行ファイルはありません。

## ノード一覧

| BT登録名（XMLで使うID） | 種別 | ライブラリ名 | 役割 |
| --- | --- | --- | --- |
| `LoggerNode` | SyncAction | `logger_node` | 指定文字列をROSログに出力 |
| `WithinGoalRadius` | Decorator | `within_goal_radius_node` | 指定フレームがゴールの半径内に入ったら子ノードを実行 |
| `RotateFwdBwdAlongTrackNode` | StatefulAction | `rotate_fwdbwd_along_track_node` | `rotate_joint` を0またはπ [rad] に向ける |

[src/rotate_node.cpp](src/rotate_node.cpp) には任意角度を指定する `RotateNode` もありますが、現在の `CMakeLists.txt` ではビルド・インストールされません。

## ファイル構成

| ファイル | 内容 |
| --- | --- |
| [src/logger_node.cpp](src/logger_node.cpp) | `LoggerNode` の実装と登録 |
| [src/within_goal_radius.cpp](src/within_goal_radius.cpp) | `WithinGoalRadius` の実装と登録 |
| [src/rotate_fwdbwd_along_track_node.cpp](src/rotate_fwdbwd_along_track_node.cpp) | `RotateFwdBwdAlongTrackNode` の実装と登録 |
| [src/rotate_node.cpp](src/rotate_node.cpp) | `RotateNode` の実装（ビルド対象外） |
| [bt_nodes.xml](bt_nodes.xml) | Grootで編集するためのノードモデル定義 |
| [CMakeLists.txt](CMakeLists.txt) | 共有ライブラリのビルド・インストール設定 |
| [package.xml](package.xml) | パッケージ情報と依存宣言 |

## 依存関係とビルド

現在のCMake設定は `behaviortree_cpp_v3` を使用します。BTを読み込む側も、対応するBehaviorTree.CPP環境を使用してください。

主な依存は `ament_cmake`、`rclcpp`、`sensor_msgs`、`com3_msgs`、`behaviortree_cpp_v3`、`pluginlib`、`nav2_msgs`、`tf2`、`tf2_geometry_msgs` です。ソースでは `geometry_msgs` と `tf2_ros` も使用しています。`package.xml` の依存宣言は一部未記載で、CMake側も含めた整理が必要です。

ROS 2の環境設定を読み込んだターミナルで、ワークスペースのルートから実行します。

```bash
colcon build --symlink-install --packages-up-to bt_control_nodes
source install/setup.bash
```

## Nav2への読み込み

本リポジトリのNav2設定では、`bt_navigator.ros__parameters.plugin_lib_names` にライブラリ名を指定します。以下の項目を、利用するノードに応じて既存リストへ追加します。

```yaml
bt_navigator:
  ros__parameters:
    plugin_lib_names:
      # 既存のNav2プラグインも保持する
      - logger_node
      - within_goal_radius_node
      - rotate_fwdbwd_along_track_node
```

設定例の [navigation_parameters.yaml](../mst110cr_navigation/params/navigation_parameters.yaml) には `logger_node` と `within_goal_radius_node` が登録されています。BT XMLで使うIDと、このライブラリ名は別です。

各ライブラリは `BT_REGISTER_NODES` でノードを登録します。Groot用の `bt_nodes.xml` を読み込むだけでは、実行時のプラグイン登録は行われません。

## LoggerNode

`text` をINFOレベルで出力し、`SUCCESS` を返します。入力を取得できない場合は警告を出力し、`FAILURE` を返します。

| 入力ポート | 型 | 既定値 | 内容 |
| --- | --- | --- | --- |
| `text` | `std::string` | なし（必須） | 出力する文字列 |

```xml
<LoggerNode text="Start navigation"/>
```

内部で `logger_node` というROSノードを生成します。tickされるたびにログを出力するため、繰り返し実行する場所ではログ量に注意してください。

## WithinGoalRadius

ゴール位置と指定フレームの位置を比較し、距離が半径以下になったら子ノードを実行するDecoratorです。子ノードを1つ指定します。

| 入力ポート | 型 | 既定値 | 内容 |
| --- | --- | --- | --- |
| `goal` | `geometry_msgs::msg::PoseStamped` | なし（必須） | 距離判定の目標位置 |
| `radius` | `double` | `1.0` | 判定半径 [m] |
| `frame` | `std::string` | `base_link` | 現在位置を取得するTFフレーム |
| `use_xy_only` | `bool` | `true` | `true` はXY平面、`false` はXYZの距離で判定 |

TFは `goal.header.frame_id` を基準に取得します。空の場合は `map` を使います。姿勢の差は判定しません。

| 状態 | 動作・戻り値 |
| --- | --- |
| 半径外 | 子を実行せず `FAILURE` |
| 必須入力を取得できない／TFを取得できない | `FAILURE` |
| 半径内 | 子を実行し、その戻り値を返す |
| 子が `RUNNING` | 距離を再判定せず子の実行を継続 |
| 子が終了／親からhaltされた | 実行開始状態をリセット |

範囲外で待機して `RUNNING` を返すノードではありません。再評価するには、親の制御ノードから再びtickされる構成にします。

Blackboardの `node`（`rclcpp::Node::SharedPtr`）と `tf_buffer`（`std::shared_ptr<tf2_ros::Buffer>`）を利用します。存在しない場合はローカルのROSノードやTFリスナーを生成します。

次は、ゴールから5 m以内なら接近ログ、それ以外なら通常ログを出す例です。`{goal}` はBlackboardに設定済みの `PoseStamped` を参照します。

```xml
<ReactiveFallback>
  <WithinGoalRadius goal="{goal}" radius="5.0"
                    frame="mst110cr_2/base_link" use_xy_only="true">
    <LoggerNode text="Near goal"/>
  </WithinGoalRadius>
  <LoggerNode text="Outside goal radius or TF unavailable"/>
</ReactiveFallback>
```

本リポジトリの [mst110cr_navigate_to_pose_w_replanning_and_recovery.xml](../mst110cr_navigation/params/mst110cr_navigate_to_pose_w_replanning_and_recovery.xml) では、ゴール付近でコントローラーを選択する処理に使用されています。

## RotateFwdBwdAlongTrackNode

`JointState` の `rotate_joint` を読み、0 radとπ radのうち現在角度との差の絶対値が小さい方を目標にします。同じ差の場合はπ radを選択します。

| 入力ポート | 型 | 既定値 | 内容 |
| --- | --- | --- | --- |
| `tolerance` | `double` | `0.1` | 到達判定の許容誤差 [rad] |

```xml
<RotateFwdBwdAlongTrackNode tolerance="0.1"/>
```

### ROSインターフェース

| 方向 | トピック | 型 |
| --- | --- | --- |
| 購読 | `/mst110cr/joint_state` | `sensor_msgs/msg/JointState` |
| 配信 | `/mst110cr/front_cmd` | `com3_msgs/msg/JointCmd` |

トピックは絶対名で固定されています。内部で `rotate_fwdbwdalongtrack_node` というROSノードを生成します。

最初のtickでは `RUNNING` を返します。その後、最初の `JointState` を受信するまでは待機し、受信後は `JointCmd.position` に目標角度を設定して配信します。`rotate_joint` 以外の位置は受信値を引き継ぎ、速度・effort配列もコピーします。`control_type` はコード内で明示設定していません。

到達誤差が `tolerance` 以下なら `SUCCESS`、それ以外は `RUNNING` を返します。指令配信はBTのtickに依存し、このノード専用の周期タイマーはありません。

### 現在の制約

- 角度差を±πに正規化していません。目標選択は単純な数値差で行うため、角度の折り返しを考慮した最短旋回ではありません。
- `target_selected_` は再実行時やhalt時にリセットされません。同じノードインスタンスでは、最初に選んだ目標が保持されます。
- `onHalted()` は空で、停止指令を送信しません。
- タイムアウト、受信データの鮮度判定、`rotate_joint` の有効な角度を受信済みかどうかの判定はありません。
- `mst110cr_com3ros` の `joint_states`／`rot_dump_cmd` とはトピック名が異なります。接続先に合わせたリマップとメッセージ内容の確認が必要です。

## Groot用ノードモデル

[bt_nodes.xml](bt_nodes.xml) は `TreeNodesModel` を含む編集用ファイルです。Nav2標準ノードとカスタムノードのモデルが記載されており、実行するBehavior Tree本体ではありません。

現在、実装との間に以下の差があります。

- `RotateFwdBwdAlongTrackNode` の `tolerance` がモデルに未記載。
- ビルド対象外の `RotateNode` がモデルに存在し、`target_angle` と `tolerance` も未記載。
- `WithinGoalRadius` のポートはありますが、型・既定値の記載がありません。
- `CMakeLists.txt` に `bt_nodes.xml` のインストール設定はありません。現状はソースディレクトリのファイルを参照します。

ポートの実際の型と既定値は、各ソースの `providedPorts()` を参照してください。

## 今後整備する項目

- `package.xml` の説明、依存関係、ライセンス宣言（現在はTODO）。
- CMake内の `plugin_libs` への重複追加の整理。
- Groot用モデルと実装のポート定義の同期。
- 旋回ノードのトピック設定、再実行・中断処理、入力検証。
- 使用するNav2／BehaviorTree.CPP環境でのビルド・BTロード・動作確認。
