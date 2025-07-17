# Mac用RViz2セットアップガイド

## 手動でRViz2を設定する方法

MacではいくつかのRVizプラグインが利用できないため、手動で表示を追加する必要があります。

### 1. IMUトラッカーを起動
```bash
cd python_control
python 2_madgwick_trajectory.py --ros2
```

### 2. RViz2を起動（別ターミナル）
```bash
# Mac対応の設定ファイルを使用
rviz2 -d rviz_config_mac.rviz

# または標準設定で起動
rviz2
```

### 3. RViz2で手動設定（標準設定で起動した場合）

#### Fixed Frameを設定
1. 左パネルの **Global Options** を選択
2. **Fixed Frame** を `base_link` に変更

#### 軌跡を表示
1. **Add** ボタンをクリック
2. **rviz_default_plugins** → **Path** を選択
3. **Topic** を `/imu/trajectory` に設定
4. **Color** を緑 (0, 255, 0) に設定
5. **Line Width** を 0.05 に設定

#### 現在位置を表示
1. **Add** ボタンをクリック
2. **rviz_default_plugins** → **Pose** を選択
3. **Topic** を `/imu/pose` に設定
4. **Color** を赤 (255, 0, 0) に設定
5. **Shape** を **Arrow** に設定

#### 速度を表示
1. **Add** ボタンをクリック
2. **rviz_default_plugins** → **TwistStamped** を選択
3. **Topic** を `/imu/velocity` に設定

#### 加速度を表示
1. **Add** ボタンをクリック
2. **rviz_default_plugins** → **AccelStamped** を選択
3. **Topic** を `/imu/acceleration` に設定

### 4. 表示の確認

#### 利用可能なトピック
```bash
# 別ターミナルで確認
ros2 topic list
```

期待される出力:
```
/imu/acceleration
/imu/data
/imu/pose
/imu/trajectory
/imu/velocity
/imu/zupt_active
/imu/cupt_active
```

#### データの確認
```bash
# 軌跡データの確認
ros2 topic echo /imu/trajectory

# 現在位置の確認
ros2 topic echo /imu/pose

# ZUPT状態の確認
ros2 topic echo /imu/zupt_active
```

### 5. トラブルシューティング

#### "No tf data"エラー
- **Fixed Frame** が `base_link` に設定されているか確認
- IMUトラッカーが正常に動作しているか確認

#### トピックが表示されない
```bash
# ROS2環境の確認
echo $ROS_DISTRO
source /opt/ros/humble/setup.bash

# ノードの確認
ros2 node list
```

#### プラグインエラー
- Mac用の設定ファイル (`rviz_config_mac.rviz`) を使用
- 利用できないプラグイン (`rviz_imu_plugin/Imu`) は使用しない

### 6. 最適な表示設定

#### カメラ視点
- **Views** パネルで **Orbit** を選択
- **Distance**: 5m
- **Pitch**: 0.5 rad
- **Yaw**: 0.785 rad

#### 色設定
- **軌跡**: 緑 (0, 255, 0) - 移動経路
- **現在位置**: 赤 (255, 0, 0) - デバイス位置
- **Grid**: グレー (160, 160, 164) - 基準

### 7. 自動起動
Mac対応の自動起動スクリプトを使用:
```bash
python launch_imu_rviz.py
```

このスクリプトは自動的にMac用設定を使用します。