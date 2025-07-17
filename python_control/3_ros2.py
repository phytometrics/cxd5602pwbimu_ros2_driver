import numpy as np
from ahrs.filters import Madgwick
from utils.imu import IMUReader
import math
import time
from collections import deque
from loguru import logger
import sys
import os
import argparse
import threading
import queue
import time
from collections import deque

# ========= 環境設定 =========================================
PORT     = "/dev/cu.usbserial-110" 
BAUDRATE = 115_200
FREQUENCY = 60.0  # IMU sampling frequency in Hz
ZUPT_THRESHOLD = 0.1  # ZUPT threshold for velocity in m/s

# ログレベルをCRITICALより上に設定（事実上ログを無効化）
logger.remove()  # デフォルトハンドラーを削除
logger.add(sys.stderr, level="CRITICAL")  # CRITICALレベルのみ

class ZUPTDetector:
    """Zero Velocity Update (ZUPT) 検出器"""
    
    def __init__(self, 
                 acc_threshold=0.2,      # 加速度の閾値 (m/s²)
                 gyro_threshold=0.05,    # ジャイロの閾値 (rad/s)
                 window_size=10,         # 判定ウィンドウサイズ
                 min_static_duration=5): # 最小静止継続フレーム数
        
        self.acc_threshold = acc_threshold
        self.gyro_threshold = gyro_threshold
        self.window_size = window_size
        self.min_static_duration = min_static_duration
        
        # データ蓄積用の循環バッファ
        self.acc_buffer = deque(maxlen=window_size)
        self.gyro_buffer = deque(maxlen=window_size)
        
        # 状態管理
        self.is_static = False
        self.static_counter = 0
        self.confidence = 0.0
        
    def add_sample(self, linear_acc, gyro):
        """新しいIMUサンプルを追加して静止判定を更新"""
        # バッファに追加
        self.acc_buffer.append(np.array(linear_acc))
        self.gyro_buffer.append(np.array(gyro))
        
        # 十分なデータが蓄積されてから判定開始
        if len(self.acc_buffer) < self.window_size:
            return self.is_static
        
        # 静止判定の実行
        acc_static = self._check_acceleration_static()
        gyro_static = self._check_gyro_static()
        magnitude_static = self._check_magnitude_static()
        
        # 総合判定（全ての条件を満たす必要がある）
        currently_static = acc_static and gyro_static and magnitude_static
        
        # 静止状態のカウンタ管理
        if currently_static:
            self.static_counter += 1
        else:
            self.static_counter = 0
        
        # 最小継続時間を満たした場合のみ静止状態とする
        self.is_static = self.static_counter >= self.min_static_duration
        
        # 信頼度の計算
        self._calculate_confidence()
        
        return self.is_static
    
    def _check_acceleration_static(self):
        """加速度の分散による静止判定"""
        if len(self.acc_buffer) == 0:
            return False
            
        acc_data = np.array(self.acc_buffer)
        
        # 各軸の分散を計算
        acc_variance = np.var(acc_data, axis=0)
        max_variance = np.max(acc_variance)
        
        return max_variance < (self.acc_threshold ** 2)
    
    def _check_gyro_static(self):
        """ジャイロの分散による静止判定"""
        if len(self.gyro_buffer) == 0:
            return False
            
        gyro_data = np.array(self.gyro_buffer)
        
        # 各軸の分散を計算
        gyro_variance = np.var(gyro_data, axis=0)
        max_variance = np.max(gyro_variance)
        
        return max_variance < (self.gyro_threshold ** 2)
    
    def _check_magnitude_static(self):
        """加速度とジャイロの大きさによる静止判定"""
        if len(self.acc_buffer) == 0 or len(self.gyro_buffer) == 0:
            return False
        
        # 最新の数サンプルの平均を使用
        recent_samples = min(3, len(self.acc_buffer))
        
        recent_acc = np.array(list(self.acc_buffer)[-recent_samples:])
        recent_gyro = np.array(list(self.gyro_buffer)[-recent_samples:])
        
        # 大きさの平均を計算
        acc_magnitudes = np.linalg.norm(recent_acc, axis=1)
        gyro_magnitudes = np.linalg.norm(recent_gyro, axis=1)
        
        avg_acc_mag = np.mean(acc_magnitudes)
        avg_gyro_mag = np.mean(gyro_magnitudes)
        
        return (avg_acc_mag < self.acc_threshold and 
                avg_gyro_mag < self.gyro_threshold)
    
    def _calculate_confidence(self):
        """静止判定の信頼度を計算 (0.0 〜 1.0)"""
        if len(self.acc_buffer) == 0:
            self.confidence = 0.0
            return
        
        # 継続時間による信頼度
        duration_confidence = min(1.0, self.static_counter / (self.min_static_duration * 2))
        
        # データの安定性による信頼度
        acc_data = np.array(self.acc_buffer)
        gyro_data = np.array(self.gyro_buffer)
        
        acc_stability = 1.0 - min(1.0, np.max(np.var(acc_data, axis=0)) / (self.acc_threshold ** 2))
        gyro_stability = 1.0 - min(1.0, np.max(np.var(gyro_data, axis=0)) / (self.gyro_threshold ** 2))
        
        # 総合信頼度
        self.confidence = (duration_confidence + acc_stability + gyro_stability) / 3.0
    
    def get_status(self):
        """現在の静止判定状態を取得"""
        return {
            'is_static': self.is_static,
            'confidence': self.confidence,
            'static_duration': self.static_counter,
            'acc_variance': np.var(np.array(self.acc_buffer), axis=0).tolist() if len(self.acc_buffer) > 0 else [0, 0, 0],
            'gyro_variance': np.var(np.array(self.gyro_buffer), axis=0).tolist() if len(self.gyro_buffer) > 0 else [0, 0, 0]
        }

class SpresenseIMUProcessor:
    """IMU処理専用クラス - ビジュアライゼーションとは完全に分離"""
    
    def __init__(self, frequency=FREQUENCY):  # gainを大幅に増加
        self.madgwick = Madgwick(frequency=frequency)
        self.q_current = np.array([1.0, 0.0, 0.0, 0.0])
        self.imu_reader = IMUReader(port=PORT, baudrate=BAUDRATE)
        # self.static_threshold = 0.001  # より小さい閾値に変更
        
        self.position = np.array([0.0, 0.0, 0.0])  # x, y, z position
        self.velocity = np.array([0.0, 0.0, 0.0])  # x, y, z velocity
        
        self.acceleration = np.array([0.0, 0.0, 0.0])  # x, y, z acceleration
        self.prev_timestamp = None
        self.dt = 1.0 / frequency
        
        # コールバック関数のリスト
        self.quaternion_callbacks = []
        
        # ZUPT検出器を追加
       # ジャイロデータ保存用
        self.current_gyro = np.array([0.0, 0.0, 0.0])
    
        self.zupt_detector = ZUPTDetector(
            acc_threshold=0.15,   # Sony Multi IMUの高精度に合わせて調整
            gyro_threshold=0.03,  # より厳密な閾値
            window_size=15,       # 60Hzで約0.25秒の窓
            min_static_duration=8 # 約0.13秒間の継続が必要
        )
        
    def add_quaternion_callback(self, callback):
        """quaternionが更新されたときに呼び出されるコールバックを追加"""
        self.quaternion_callbacks.append(callback)
        
    def remove_quaternion_callback(self, callback):
        """コールバックを削除"""
        if callback in self.quaternion_callbacks:
            self.quaternion_callbacks.remove(callback)
    
    def _notify_quaternion_update(self, q, timestamp, raw_data):
        """quaternion更新を全てのコールバックに通知"""
        for callback in self.quaternion_callbacks:
            try:
                callback(q, timestamp, raw_data)
            except Exception as e:
                print(f"Callback error: {e}")
    
    def process_imu_data(self, sec, usec, ax, ay, az, gx, gy, gz):
        """IMUデータを処理してquaternionを更新"""
        acc_ms2 = np.array([ax, ay, az])
        gyro_rads = np.array([gx, gy, gz])
        
        # ジャイロデータをクラス変数として保存（ZUPT用）
        self.current_gyro = gyro_rads.copy()
        
        self.q_current = self.madgwick.updateIMU(
            self.q_current, gyr=gyro_rads, acc=acc_ms2
        )
        
        # コールバックに通知
        timestamp = sec * 1_000_000 + usec   # 起動からのマイクロ秒
        trajectory_data = self._update_trajectory(acc_ms2, timestamp)
        
        raw_data = {
            'acc': [ax, ay, az],
            'gyro': [gx, gy, gz],
            'euler': self.quaternion_to_euler(self.q_current),
            'trajectory': trajectory_data
        }
        logger.info(f"Data: {raw_data}")
        self._notify_quaternion_update(self.q_current, timestamp, raw_data)
        
        return self.q_current

    
    def rotate_vector_by_quaternion(self, vector, quaternion):
        """
        クォータニオンを使用してベクトルを回転
        
        Args:
            vector: 回転させるベクトル [x, y, z]
            quaternion: 回転を表すクォータニオン [w, x, y, z] (正規化済み)
        
        Returns:
            回転後のベクトル
        """
        q_w, q_x, q_y, q_z = quaternion
        v_x, v_y, v_z = vector
        
        # 方法1: 直接的な公式を使用
        # v' = v + 2 * cross(q_xyz, cross(q_xyz, v) + q_w * v)
        q_xyz = np.array([q_x, q_y, q_z])
        v = np.array([v_x, v_y, v_z])
        
        cross1 = np.cross(q_xyz, v)
        cross2 = np.cross(q_xyz, cross1 + q_w * v)
        rotated_vector = v + 2 * cross2
        return rotated_vector

    def _update_trajectory(self, acceleration, timestamp):
        if self.prev_timestamp is None:
            self.prev_timestamp = timestamp
            return {
                'position': self.position.copy(),
                'velocity': self.velocity.copy(),
                'acceleration': self.acceleration.copy(),
                'zupt_status': {'is_static': False, 'confidence': 0.0}
            }
        
        """IMUの加速度データを使用して軌跡を更新"""
        dt = (timestamp - self.prev_timestamp) / 1000000.0  # マイクロ秒から秒に変換
        
        if dt <= 0:
            logger.warning("Invalid timestamp difference, skipping trajectory update")
            return {
                'position': self.position.copy(),
                'velocity': self.velocity.copy(),
                'acceleration': self.acceleration.copy(),
                'zupt_status': {'is_static': False, 'confidence': 0.0}
            }

        # センサ系における重力ベクトルを算出
        acc_sensor = np.array(acceleration)
        q_conj = np.array([self.q_current[0], -self.q_current[1], -self.q_current[2], -self.q_current[3]])
        g_sensor = self.rotate_vector_by_quaternion(
            np.array([0.0, 0.0, 9.81]),  # 世界系重力（下向き）※座標系によって符号は調整
            q_conj                       # world → sensor
        )
        # 生の加速度から重力を引く（still in センサ系）
        lin_acc_sensor = acc_sensor - g_sensor

        # 線形加速度を世界系に回転
        self.acceleration = self.rotate_vector_by_quaternion(
            lin_acc_sensor,
            self.q_current             # sensor → world
        )
        # self.acceleration = linear_acc
        # 加速度をquaternion使ってワールド座標系に変換
        # self.acceleration = self.rotate_vector_by_quaternion(acceleration, self.q_current)
        # # q_conj = np.array([self.q_current[0], -self.q_current[1], -self.q_current[2], -self.q_current[3]])
        # gravity = np.array([0.0, 0.0, +9.81])        
        
        # gravity = self.rotate_vector_by_quaternion(gravity, self.q_current)
        # linear_acc = self.acceleration - gravity  # 重力除去済み線形加速度
        
        # ジャイロデータを取得（process_imu_dataから渡す必要がある）
        # 現在のコードではジャイロデータが_update_trajectoryに渡されていないので、
        # クラス変数として保存する方法を使用
        if hasattr(self, 'current_gyro'):
            gyro_data = self.current_gyro
        else:
            # フォールバック：ゼロベクトル（理想的ではないが安全）
            gyro_data = np.array([0.0, 0.0, 0.0])
            logger.warning("Gyro data not available for ZUPT, using zero vector")
        
        # ZUPT検出器で静止判定
        is_static = self.zupt_detector.add_sample(self.acceleration, gyro_data)
        zupt_status = self.zupt_detector.get_status()
        
        # 前の速度を保存
        prev_velocity = self.velocity.copy()
        
        # 通常の速度更新: v = v0 + a * dt
        new_velocity = self.velocity + self.acceleration * dt
        
        # ZUPT適用
        if is_static:
            # 静止状態: 速度をゼロに向かって収束させる
            confidence = zupt_status['confidence']
            zupt_strength = 0.95  # ZUPT適用強度
            correction_factor = zupt_strength * confidence
            
            # 速度を段階的にゼロに収束
            new_velocity = new_velocity * (1.0 - correction_factor)
            
            # 非常に小さい値は完全にゼロにする
            threshold = 0.001  # 1mm/s
            new_velocity = np.where(np.abs(new_velocity) < threshold, 0.0, new_velocity)
            
            # logger.debug(f"ZUPT適用: 信頼度={confidence:.3f}, 補正後速度={np.linalg.norm(new_velocity):.6f} m/s")
        
        self.velocity = new_velocity
        
        # 位置更新: p = p0 + (v0 + v1)/2 * dt (台形積分)
        avg_velocity = (prev_velocity + self.velocity) / 2.0
        self.position += avg_velocity * dt
        
        self.prev_timestamp = timestamp
        
        # デバッグ情報
        # if zupt_status['is_static']:
        #     logger.info(f"ZUPT適用中 (信頼度: {zupt_status['confidence']:.3f}, 継続:{zupt_status['static_duration']}フレーム)")

        return {
            'position': self.position.copy(),
            'velocity': self.velocity.copy(),
            'acceleration': self.acceleration.copy(),  # 重力除去済みの加速度を返す
            'zupt_status': zupt_status
        }
    def quaternion_to_euler(self, q):
        q_w, q_x, q_y, q_z = q
        # Roll (x-axis rotation)
        roll = math.atan2(2 * (q_w * q_x + q_y * q_z), 1 - 2 * (q_x**2 + q_y**2))        
        # Pitch (y-axis rotation)
        pitch = math.asin(2 * (q_w * q_y - q_z * q_x))
        # Yaw (z-axis rotation)
        yaw = math.atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y**2 + q_z**2))
        # output in radians
        return roll, pitch, yaw
    
    def run_realtime(self, enable_ros2=False):
        """リアルタイムIMU処理を開始"""
        print("Starting Madgwick filter with firmware-calibrated IMU data...")
        print("Press Ctrl+C to stop")
        
        ros2_bridge = None
        if enable_ros2:
            try:
                from utils.ros2_imu_publisher import IMUTrajectoryROS2Bridge
                ros2_bridge = IMUTrajectoryROS2Bridge()
                self.add_quaternion_callback(ros2_bridge.quaternion_callback)
                ros2_bridge.start_spinning()
                logger.info("ROS2 publishing enabled")
            except ImportError as e:
                logger.warning(f"ROS2 not available: {e}")
                enable_ros2 = False
            except Exception as e:
                logger.error(f"Failed to initialize ROS2: {e}")
                enable_ros2 = False
        
        try:
            with self.imu_reader as reader:
                for sec, usec, ax, ay, az, gx, gy, gz in reader.stream_data():
                    q = self.process_imu_data(sec, usec, ax, ay, az, gx, gy, gz)
                    roll, pitch, yaw = self.quaternion_to_euler(q)
                    print(f"corrected acceleration: {self.acceleration}")
                    # print(f"Quaternion: {q}")
                    # print(f"{sec}.{usec:03d} "
                    #       f"acc={ax:+.3f},{ay:+.3f},{az:+.3f} (m/sec2)"
                    #       f"gyro={gx:+.3f},{gy:+.3f},{gz:+.3f} (radians/sec)"
                    #       f"q=({q[0]:+.3f},{q[1]:+.3f},{q[2]:+.3f},{q[3]:+.3f}) "
                    #       f"rpy=({math.degrees(roll):+6.1f}°,{math.degrees(pitch):+6.1f}°,{math.degrees(yaw):+6.1f}°)")
                    # loguru version
                    # logger.info(f"{sec}.{usec:03d} ALMOST RAW:"
                    #             f"acc={ax:+.3f},{ay:+.3f},{az:+.3f} (m/sec2)"
                    #             f"gyro={gx:+.3f},{gy:+.3f},{gz:+.3f} (radians/sec )"
                    #             f"q=({q[0]:+.3f},{q[1]:+.3f},{q[2]:+.3f},{q[3]:+.3f}) ")
                                # f"rpy=({math.degrees(roll):+6.1f}°,{math.degrees(pitch):+6.1f}°,{math.degrees(yaw):+6.1f}°)")
        except KeyboardInterrupt:
            print("\nStopping IMU processing...")
        finally:
            if ros2_bridge:
                ros2_bridge.stop_spinning()

def main(args):
    """メイン実行関数"""
    print("Starting IMU Visualizer...")
        
    # IMU処理クラスを作成
    imu_processor = SpresenseIMUProcessor(
        frequency=FREQUENCY
    )
    # IMU処理を開始（メインスレッドで実行）
    try:
        imu_processor.run_realtime(enable_ros2=args.ros2)
    except KeyboardInterrupt:
        print("\nShutting down...")

if __name__ == "__main__":
    args = argparse.ArgumentParser(description="IMU Visualizer with Madgwick filter")
    args.add_argument('--ros2', action='store_true',
                      help='Enable ROS2 publishing for IMU data')
    args = args.parse_args()
    main(args)