# cxd5602pwbimu_ros2_driver

ROS 2 driver for Multi-IMU Add-on board for SONY SPRESENSE™

[![ci_humble](https://github.com/NITKK-ROS-Team/cxd5602pwbimu_ros2_driver/actions/workflows/ci_humble.yml/badge.svg)](https://github.com/NITKK-ROS-Team/cxd5602pwbimu_ros2_driver/actions/workflows/ci_humble.yml)


## 概要

このパッケージは、SONY SPRESENSE™にMulti-IMUアドオンボードを搭載したCXD5602PWBIMUからIMUデータを取得し、ROS 2トピックとして配信するドライバです。

## 機能

- CXD5602PWBIMUからのIMUデータ（加速度、角速度）の取得
- IMUフィルタリング（Madgwick filter）
- RVizでのデータ可視化
- シリアル通信による高速データ転送

## 必要なハードウェア

- SONY SPRESENSE™ メインボード
- Multi-IMU Add-on board
- USBケーブル

## 必要なソフトウェア

- ROS 2 Humble
- h6x_serial_interface
- imu_filter_madgwick
- rviz2

## インストール

### 1. 依存関係のインストール

```bash
# ROS 2 Humbleがインストールされていることを確認
source /opt/ros/humble/setup.bash

# 依存パッケージのインストール
sudo apt install ros-humble-imu-filter-madgwick
```

### 2. ワークスペースの作成

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 3. パッケージのクローン

```bash
git clone https://github.com/NITKK-ROS-Team/cxd5602pwbimu_ros2_driver.git
cd ~/ros2_ws
```

### 4. 依存関係の取得

```bash
vcs import src < src/cxd5602pwbimu_ros2_driver/build_depends.repos
```

### 5. ビルド

```bash
colcon build --packages-select cxd5602pwbimu_driver cxd5602pwbimu_driver_node cxd5602pwbimu_driver_bringup
```

### 6. セットアップ

```bash
source install/setup.bash
```

## ファームウェアのセットアップ

### 1. SPRESENSE側のファームウェア

`firmware/cxd5602pwbimu.ino`をSPRESENSE Arduino IDEで開き、ボードに書き込みます。

### 2. デバイスの接続

SPRESENSEをUSBケーブルでPCに接続し、デバイスパスを確認してください（通常は`/dev/ttyUSB0`）。

## 使用方法

### 1. 基本的な起動

```bash
ros2 launch cxd5602pwbimu_driver_bringup cxd5602pwbimu.launch.py
```

### 2. デバイスパスの指定

```bash
ros2 launch cxd5602pwbimu_driver_bringup cxd5602pwbimu.launch.py device:=/dev/ttyUSB0
```

### 3. 個別ノードの起動

```bash
# IMUドライバノードのみ起動
ros2 run cxd5602pwbimu_driver_node cxd5602pwbimu_driver_node_exec --ros-args -p device:=/dev/ttyUSB0

# IMUフィルタのみ起動
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args -p use_mag:=true device:=/dev/ttyUSB0

# RVizでの可視化
ros2 run rviz2 rviz2 -d src/cxd5602pwbimu_driver_bringup/rviz/rvizconfig.rviz
```

## トピック

### Published Topics

- `/imu/data_raw` (sensor_msgs/msg/Imu): 生のIMUデータ
- `/imu/data` (sensor_msgs/msg/Imu): フィルタリング後のIMUデータ

### Parameters

- `device` (string, default: "/dev/ttyUSB0"): シリアルデバイスのパス
- `baudrate` (int, default: 115200): シリアル通信のボーレート
- `timeout_ms` (int, default: 100): シリアル通信のタイムアウト
- `use_mag` (bool, default: false): 磁気センサーの使用有無

## トラブルシューティング

### デバイスが認識されない

```bash
# デバイスの確認
ls -la /dev/ttyUSB*

# 権限の確認
sudo chmod 666 /dev/ttyUSB0
```

### データが受信できない

1. SPRESENSEのファームウェアが正しく書き込まれているか確認
2. デバイスパスが正しいか確認
3. ボーレートが一致しているか確認（デフォルト: 115200）

### ビルドエラー

```bash
# 依存関係の再インストール
rosdep install --from-paths src --ignore-src -r -y

# クリーンビルド
rm -rf build install log
colcon build
```

## ライセンス

Apache License 2.0

## 作者

- Maintainer: Ar-Ray-code (ray255ar@gmail.com)
- Organization: NITK.K ROS-Team

## 貢献

Issues, Pull Requestsを歓迎します。

## 関連リンク

- [SONY SPRESENSE™ 公式サイト](https://developer.sony.com/develop/spresense/)
- [h6x_serial_interface](https://github.com/HarvestX/h6x_serial_interface)
- [imu_filter_madgwick](https://github.com/ccny-ros-pkg/imu_tools)

## 異なるPCでの使用について

ファームウェアの書き込みとROS 2ドライバの実行は、異なるPCで行うことができます：

1. **PC1（書き込み用）**: Arduino IDEを使用してSPRESENSEにファームウェアを書き込み
2. **PC2（ROS用）**: ファームウェアが書き込まれたSPRESENSEをUSB接続してROS 2ドライバを実行

SPRESENSEのファームウェアは一度書き込まれると保持されるため、別のPCに接続してもそのまま動作します。