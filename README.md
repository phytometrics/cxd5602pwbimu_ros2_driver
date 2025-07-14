# Multi-IMU Add-on Board driver
ROS 2 driver for Multi-IMU Add-on board for SONY SPRESENSE

## Recent Updates
- **2025-01-14**: Fixed timestamp synchronization issues causing filter resets
- **2025-01-14**: Added ROS 2 Humble compatibility for message_filters
- **2025-01-14**: Improved IMU data filtering stability

# Firmware Install

- Use Arduino IDE
## 公式の手順に従って環境設定を行う
- https://developer.sony.com/spresense/development-guides/arduino_set_up_ja　(1.3. USB ドライバのインストールまででよい)
- ファームウェア書き込みに限っては実際にMulti-IMU Add-on Boardを使う装置でなくてもよい。

## ファームウェア書き込み

CRCライブラリ有効化
- Sketch - Include Library - Manage Libraries
CRC by Rob Tillaartを検索し、インストール

- `firmware/cxd5602pwbimu.ino`をSPRESENSE Arduino IDEで開き（もしくはコピペ貼り付け）、verifyでエラーがないこと確認したあと、ボードに書き込む。

```
####################################
Sketch uses 183168 bytes (23%) of program storage space. Maximum is 786432 bytes.
Global variables use 183168 bytes (23%) of dynamic memory, leaving 603264 bytes for local variables. Maximum is 786432 bytes.
## Used memory size:  768 [KByte] ##
####################################
>>> Install files ...
install
Install /Users/yosuke/Library/Caches/arduino/sketches/9258F2FADA7DAE3C56672E538122DEE9/spresense.ino.spk
|0%-----------------------------50%------------------------------100%|
######################################################################

170896 bytes loaded.
Package validation is OK.
Saving package to "nuttx"
sync
Restarting the board ...
reboot
```

## ポート調べる
dmesg | grep tty
ls /dev/ttyUSB*

```
root@ubuntu:~/ros2_ws/src/spresense# dmesg | grep tty
```
```
[    0.000000] Kernel command line: root=PARTUUID=19274313-2e65-42a1-b5e1-6a
....
**[ 2848.925324] usb 1-3.2: cp210x converter now attached to ttyUSB0**

```
## 追加インストール必要なもの

libSerial
```
git clone https://github.com/crayzeewulf/libserial.git
cd libserial
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig
```
Madgwickフィルタ
```
sudo apt install ros-humble-imu-filter-madgwick
```

#　インストール（humble環境）

## 注意
- docker環境の場合、docker exec -it --privilegedをつけて実行すること

```
git clone THIS_REPO
cd cxd5602pwbimu_ros2_driver
colcon build
```
## 起動
rviz2も起動する。
```
source install/setup.bash
ros2 launch cxd5602pwbimu_driver_bringup cxd5602pwbimu.launch.py dev:=/dev/PATH_TO_DEVICE
```

## IMU Filtering
IMU軌跡フィルタを使用してデータを平滑化:
```
ros2 launch imu_trajectory_filter imu_trajectory_filter.launch.py device:=/dev/ttyUSB0
```

## RViz2で軌跡表示
```
rviz2 -d src/imu_trajectory_filter/rviz/trajectory_view.rviz
```

# 値
## Multi-IMU Add-on Board raw output
静置時の値
```
python get_output.py
```
```
4.265  acc=(+2.227,+0.417,-9.566)  gyro=(+0.002,+0.007,-0.000)
4.268  acc=(+2.225,+0.434,-9.589)  gyro=(+0.003,-0.001,+0.001)
4.271  acc=(+2.193,+0.423,-9.570)  gyro=(+0.004,-0.003,+0.000)
```
## ros2 topic of Multi-IMU
```
root@ubuntu:~/ros2_ws# ros2 topic echo /imu/data --once
header:
  stamp:
    sec: 1752136201
    nanosec: 129000000
  frame_id: imu
orientation:
  x: -0.08054075874348365
  y: -0.9773934515702924
  z: -0.08728528062213273
  w: 0.17491857189918297
orientation_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
angular_velocity:
  x: 0.008584074676036835
  y: -0.0011573940282687545
  z: 0.006373170297592878
angular_velocity_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
linear_acceleration:
  x: 2.3165674209594727
  y: 1.2016679048538208
  z: -9.34932804107666
linear_acceleration_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
---
```
```
root@ubuntu:~/ros2_ws# ros2 topic echo /imu/data_raw --once
header:
  stamp:
    sec: 1752136338
    nanosec: 424000000
  frame_id: imu
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
orientation_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
angular_velocity:
  x: -0.004437691532075405
  y: -0.002792589133605361
  z: 0.011386241763830185
angular_velocity_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
linear_acceleration:
  x: 3.5480008125305176
  y: 2.772167682647705
  z: -9.158851623535156
linear_acceleration_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
---
```
## unitree go2 mcu imu
https://techshare.co.jp/faq/unitree/mid360-on-go2_fast-lio.html#3-2_FAST-LIOIMU
```
ros2 topic echo /go2/imu
```
```
---
header:
  stamp:
    sec: 1752134693
    nanosec: 435021447
  frame_id: imu_link
orientation:
  x: 0.041872527450323105
  y: -0.03419948369264603
  z: 0.6368061304092407
  w: 0.7691263556480408
orientation_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
angular_velocity:
  x: 0.0021305286791175604
  y: -0.004261057358235121
  z: -0.012783171609044075
angular_velocity_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
linear_acceleration:
  x: 0.8343793153762817
  y: -0.17477671802043915
  z: 9.678560256958008
linear_acceleration_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
```


## unitree go2 imu (utlidar/imu)
```
ros2 topic echo /utlidar/imu
```
静置時の値
```
header:
  stamp:
    sec: 1752134696
    nanosec: 538722515
  frame_id: utlidar_imu
orientation:
  x: 0.735081136226654
  y: 0.6516342163085938
  z: -0.026073023676872253
  w: -0.1760013997554779
orientation_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
angular_velocity:
  x: 0.013672907836735249
  y: -0.01381600834429264
  z: 0.006575479172170162
angular_velocity_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
linear_acceleration:
  x: 1.9827041625976562
  y: -2.7919187545776367
  z: -9.175609588623047
linear_acceleration_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
---
```


# Mac installの場合
- libserialのビルドをこのレポジトリ外でやる。
- brew install doxygen.



```
git clone https://github.com/ccny-ros-pkg/imu_tools.git -b humble
cd ..
rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y --skip-keys="libserial-dev ros-humble-ament-clang-format ros-humble-ament-clang-tidy"
```

## ビルド
conda activate ros_env
cd /Users/yosuke/gits/cxd5602pwbimu_ros2_driver/ros2_ws
colcon build --packages-select imu_trajectory_filter

## 実行
source install/setup.zsh
<!-- インストールツリー直下の lib をすべて動的リンク検索パスに追加 -->
export DYLD_LIBRARY_PATH=$PWD/install/imu_trajectory_filter/lib:$PWD/install/h6x_serial_interface/lib:$PWD/install/LibSerial/lib:${DYLD_LIBRARY_PATH}
export DYLD_LIBRARY_PATH=/usr/local/lib:${DYLD_LIBRARY_PATH}


ros2 launch imu_trajectory_filter imu_trajectory_filter.launch.py device:=/dev/ttyUSB0

## RViz2で軌跡表示
rviz2 -d src/imu_trajectory_filter/rviz/trajectory_view.rviz

## Known Issues and Solutions
### Large timestamp gap warnings
以前のバージョンでは、ファームウェアのタイムスタンプが不正確でフィルタリングが機能しない問題がありました。
現在のバージョンでは、ROS2の現在時刻を使用してこの問題を解決しています。

### ROS 2 Humble compatibility
imu_tools submoduleでmessage_filters::Subscriberのコンストラクタに関する互換性問題が修正されています。

lsof | grep tty.usbserial


colcon build --cmake-args \
  -DCMAKE_PREFIX_PATH=/usr/local \
  -DCMAKE_CXX_FLAGS="-I/usr/local/include" \
  -DCMAKE_EXE_LINKER_FLAGS="-L/usr/local/lib"




  ２回目移行にやるやつ

  ```
  readlink -f /sys/class/tty/ttyUSB0               # 例: /sys/class/tty/ttyUSB0 -> ../../devices/pci0000:00/0000:00:14.0/usb1/1-3/1-3:1.0/ttyUSB0
# 末尾の 1-3:1.0 がデバイス ID
  ```

  ```
  ID="1-3.2:1.0"                 # 上で得た ID
echo -n "$ID" | sudo tee /sys/bus/usb/drivers/usb/unbind
echo -n "$ID" | sudo tee /sys/bus/usb/drivers/usb/bind
  ```

  ```
alias usbreset='ID=$(readlink -f /sys/class/tty/ttyUSB0 | sed "s|.*/||"); echo -n $ID | sudo tee /sys/bus/usb/drivers/usb/unbind && echo -n $ID | sudo tee /sys/bus/usb/drivers/usb/bind'
  ```


  sudo apt install ros-humble-rviz-imu-plugin 