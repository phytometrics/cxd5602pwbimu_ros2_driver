# RViz2でIMU軌跡を可視化する方法

## 必要な準備

### 1. ROS2のインストール
```bash
# Ubuntu/Debian (ROS2 Humble推奨)
sudo apt update
sudo apt install ros-humble-desktop ros-humble-rviz-imu-plugin

# macOS (Homebrewを使用)
brew install ros
```

### 2. ROS2環境の設定
```bash
# 毎回実行が必要
source /opt/ros/humble/setup.bash

# または .bashrc に追加
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 3. 必要なPythonパッケージ
```bash
pip install rclpy sensor_msgs geometry_msgs nav_msgs std_msgs
```

## 使用方法

### 方法1: 自動起動スクリプト（推奨）
```bash
# RViz2とIMUトラッカーを同時起動
python launch_imu_rviz.py
```

### 方法2: 手動で起動

#### ターミナル1: IMUトラッカーを起動
```bash
# ROS2有効でIMUトラッカーを起動
python 2_madgwick_trajectory.py --ros2

# デバッグモードで詳細表示
LOG_LEVEL=DEBUG python 2_madgwick_trajectory.py --ros2

# ZUPT無効で起動
python 2_madgwick_trajectory.py --ros2 --no-zupt
```

#### ターミナル2: RViz2を起動
```bash
# 設定ファイル付きで起動
rviz2 -d rviz_config.rviz

# または標準設定で起動
rviz2
```

## RViz2での表示設定

### 1. 基本設定
- **Fixed Frame**: `base_link` に設定
- **Grid**: XY平面に1mグリッド表示

### 2. 表示要素

#### IMUデータ (`/imu/data`)
- **表示内容**: IMUの向き（3D矢印）
- **色**: デフォルト（白/赤）
- **スケール**: 1.0
- **機能**: デバイスの回転状態をリアルタイム表示

#### 軌跡 (`/imu/trajectory`)
- **表示内容**: 移動経路の線
- **色**: 緑 (25, 255, 0)
- **線幅**: 0.03m
- **バッファ**: 1000点まで保持

#### 現在位置 (`/imu/pose`)
- **表示内容**: 現在位置と向き（矢印）
- **色**: 赤 (255, 25, 0)
- **サイズ**: 矢印長さ1m

#### 速度 (`/imu/velocity`)
- **表示内容**: TwistStamped（線速度）
- **用途**: 数値確認（プロパティパネル）

#### 加速度 (`/imu/acceleration`)
- **表示内容**: AccelStamped（加速度）
- **用途**: 数値確認（プロパティパネル）

#### ZUPT/CUPT状態
- **`/imu/zupt_active`**: 速度リセット時にTrue
- **`/imu/cupt_active`**: 位置リセット時にTrue
- **用途**: デバッグ時の状態確認

### 3. 表示の追加・設定変更

#### 新しい表示を追加:
1. **Add** ボタンをクリック
2. 表示タイプを選択（Path, PoseStamped, Imu等）
3. **Topic** でトピック名を設定
4. **Color**, **Scale** 等を調整

#### 既存表示の設定変更:
1. 左パネルで表示項目を選択
2. プロパティを変更
   - **Color**: 色を変更
   - **Scale**: サイズを変更
   - **Alpha**: 透明度を変更

## トラブルシューティング

### RViz2が起動しない
```bash
# ROS2環境の確認
echo $ROS_DISTRO
which rviz2

# 環境設定の再実行
source /opt/ros/humble/setup.bash
```

### トピックが表示されない
```bash
# トピック一覧の確認
ros2 topic list

# 特定トピックの確認
ros2 topic echo /imu/data
ros2 topic hz /imu/trajectory
```

### IMUデータが来ない
```bash
# IMUトラッカーのログ確認
LOG_LEVEL=DEBUG python 2_madgwick_trajectory.py --ros2

# シリアルポートの確認
ls /dev/cu.usbserial-*    # macOS
ls /dev/ttyUSB*           # Linux
```

### フレームエラー
```bash
# TFの確認
ros2 run tf2_tools view_frames.py

# Fixed Frameをbase_linkに設定
# Global Options > Fixed Frame: base_link
```

## 使用例とコマンド集

### デバッグモードで全機能表示
```bash
LOG_LEVEL=DEBUG python 2_madgwick_trajectory.py --ros2
```

### ZUPT無効で高感度追跡
```bash
python 2_madgwick_trajectory.py --ros2 --no-zupt --frequency 100
```

### カスタム閾値で実行
```bash
python 2_madgwick_trajectory.py --ros2 --zupt-vel-threshold 0.05 --cupt-pos-threshold 5.0
```

### 軌跡のクリア（実行中）
```bash
# 新しいターミナルで
ros2 service call /clear_trajectory std_srvs/srv/Empty
```

## 表示のカスタマイズ

### 色の変更
- **軌跡**: 緑 → 青に変更
  1. Trajectory > Color をクリック
  2. RGB値を (0, 0, 255) に設定

### 表示範囲の調整
- **Grid**: Cell Size を 0.5m に変更
- **Camera**: Distance を 5m に調整

### 追加表示
- **Velocity vectors**: 速度ベクトルの矢印表示
- **Acceleration arrows**: 加速度の可視化
- **Text displays**: 数値の文字表示

これらの設定により、IMUの軌跡追跡を直感的に確認できます。