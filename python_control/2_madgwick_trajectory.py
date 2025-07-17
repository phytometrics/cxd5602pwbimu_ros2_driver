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



# ========= 環境設定 =========================================
PORT     = "/dev/cu.usbserial-110" 
BAUDRATE = 115_200

# ログレベル設定 (環境変数またはデフォルト)
LOG_LEVEL = os.getenv('LOG_LEVEL', 'INFO')

# loguru設定
logger.remove()  # デフォルトハンドラを削除
logger.add(
    sys.stderr,
    level=LOG_LEVEL,
    format="<green>{time:YYYY-MM-DD HH:mm:ss.SSS}</green> | <level>{level: <8}</level> | <cyan>{name}</cyan>:<cyan>{function}</cyan>:<cyan>{line}</cyan> - <level>{message}</level>"
)

# 静的閾値をクラス外で定義
STATIC_THRESHOLD = 0.05

# ZUPT/CUPT設定（より敏感な閾値に調整）
ZUPT_ENABLED = os.getenv('ZUPT_ENABLED', 'True').lower() == 'true'
CUPT_ENABLED = os.getenv('CUPT_ENABLED', 'True').lower() == 'true'
ZUPT_VELOCITY_THRESHOLD = float(os.getenv('ZUPT_VELOCITY_THRESHOLD', '0.01'))  # 0.1 -> 0.01
ZUPT_ACCEL_THRESHOLD = float(os.getenv('ZUPT_ACCEL_THRESHOLD', '0.2'))       # 0.5 -> 0.2  
CUPT_POSITION_THRESHOLD = float(os.getenv('CUPT_POSITION_THRESHOLD', '1.0'))  # 10.0 -> 2.0

class SpresenseIMUProcessor:

    def __init__(self, frequency=60.0, zupt_enabled=ZUPT_ENABLED, cupt_enabled=CUPT_ENABLED):
        self.madgwick = Madgwick(frequency=frequency)
        self.q_current = np.array([1.0, 0.0, 0.0, 0.0])
        self.imu_reader = IMUReader(port=PORT, baudrate=BAUDRATE)        
        # コールバック関数のリスト
        self.quaternion_callbacks = []
        self.static_threshold = STATIC_THRESHOLD
        
        # 軌跡計算用の状態変数
        self.position = np.array([0.0, 0.0, 0.0])  # x, y, z position
        self.velocity = np.array([0.0, 0.0, 0.0])  # x, y, z velocity
        self.acceleration = np.array([0.0, 0.0, 0.0])  # x, y, z acceleration
        self.prev_timestamp = None
        self.dt = 1.0 / frequency
        
        # バイアス補正用
        self.accel_bias = np.array([0.0, 0.0, 0.0])  # 加速度バイアス
        self.bias_estimation_count = 0
        self.bias_estimation_samples = int(frequency * 3)  # 3秒間でバイアス推定
        
        # ZUPT/CUPT設定
        self.zupt_enabled = False
        self.cupt_enabled = False
        # self.zupt_enabled = zupt_enabled
        # self.cupt_enabled = cupt_enabled
        self.zupt_velocity_threshold = ZUPT_VELOCITY_THRESHOLD
        self.zupt_accel_threshold = ZUPT_ACCEL_THRESHOLD
        self.cupt_position_threshold = CUPT_POSITION_THRESHOLD
        
        # ZUPT/CUPT状態追跡
        self.velocity_history = deque(maxlen=10)  # 速度履歴（ZUPT判定用）
        self.accel_history = deque(maxlen=10)     # 加速度履歴（ZUPT判定用）
        self.zupt_counter = 0
        self.initial_position = None
        
        logger.info(f"IMUProcessor initialized with frequency={frequency}, static_threshold={self.static_threshold}")
        logger.info(f"ZUPT enabled: {self.zupt_enabled}, CUPT enabled: {self.cupt_enabled}")
        logger.info(f"Bias estimation will run for {self.bias_estimation_samples} samples (~3 seconds)")
        logger.info("Please keep the device stationary during bias estimation period")
        
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
                logger.error(f"Callback error: {e}")
    
    def process_imu_data(self, sec, usec, ax, ay, az, gx, gy, gz):
        """IMUデータを処理してquaternionを更新"""
        # Debug: raw gyro values to check for non-zero data
        if abs(gx) > 0.01 or abs(gy) > 0.01 or abs(gz) > 0.01:
            logger.debug(f"Non-zero gyro detected: gx={gx:.6f}, gy={gy:.6f}, gz={gz:.6f}")
        
        acc_ms2 = np.array([ax, ay, az])
        # ジャイロは度/秒で来るので、ラジアン/秒に変換 ->最初からrad/secっぽい
        gyro_rads = np.array([gx, gy, gz]) #* np.pi / 180.0
        
        # Debug: Check if conversion causes zeros
        if np.any(np.abs(gyro_rads) > 0.001):
            logger.debug(f"gyro_rads = {gyro_rads}")
        
        # ジャイロマグニチュードを計算
        gyro_magnitude = np.linalg.norm(gyro_rads)
        logger.debug(f"gyro_magnitude = {gyro_magnitude:.6f}, threshold = {self.static_threshold}")
        
        # 静的検出の条件を緩和し、動的なgain調整
        if gyro_magnitude < self.static_threshold:
            # 静的条件でも適度なgainを維持
            temp_gain = self.madgwick.gain
            self.madgwick.gain = 0.01  # 0.0005から0.01に増加
            logger.debug("Using static gain (0.01)")
            self.q_current = self.madgwick.updateIMU(
                self.q_current, gyr=gyro_rads, acc=acc_ms2
            )
            self.madgwick.gain = temp_gain  # Restore original gain
        else:
            logger.debug(f"Using dynamic gain ({self.madgwick.gain})")
            self.q_current = self.madgwick.updateIMU(
                self.q_current, gyr=gyro_rads, acc=acc_ms2
            )
        
        # 軌跡計算
        timestamp = sec + usec / 1000000.0
        trajectory_data = self._update_trajectory(acc_ms2, timestamp)
        
        # コールバックに通知
        timestamp_str = f"{sec}.{usec:03d}"
        raw_data = {
            'acc': [ax, ay, az],
            'gyro': [gx, gy, gz],
            'euler': self.quaternion_to_euler(self.q_current),
            'trajectory': trajectory_data
        }
        self._notify_quaternion_update(self.q_current, timestamp_str, raw_data)
        
        return self.q_current
    
    def quaternion_to_euler(self, q):
        """quaternionをオイラー角に変換"""
        w, x, y, z = q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def _rotate_vector_by_quaternion(self, vector, quaternion):
        """quaternionを使ってベクトルを回転（修正版）"""
        w, x, y, z = quaternion
        vx, vy, vz = vector
        
        # 正しいquaternion rotation formula (q * v * q^-1)
        # 回転行列を使った実装
        # R = I + 2*skew(q_xyz) + 2*skew(q_xyz)^2 / |q|^2
        
        # quaternionの正規化（安全のため）
        q_norm = np.sqrt(w*w + x*x + y*y + z*z)
        if q_norm > 0:
            w, x, y, z = w/q_norm, x/q_norm, y/q_norm, z/q_norm
        
        # 回転行列の計算
        xx, yy, zz = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        wx, wy, wz = w*x, w*y, w*z
        
        # 回転行列の各要素
        r11 = 1 - 2*(yy + zz)
        r12 = 2*(xy - wz)
        r13 = 2*(xz + wy)
        
        r21 = 2*(xy + wz)
        r22 = 1 - 2*(xx + zz)
        r23 = 2*(yz - wx)
        
        r31 = 2*(xz - wy)
        r32 = 2*(yz + wx)
        r33 = 1 - 2*(xx + yy)
        
        # ベクトルの回転
        rotated_x = r11 * vx + r12 * vy + r13 * vz
        rotated_y = r21 * vx + r22 * vy + r23 * vz
        rotated_z = r31 * vx + r32 * vy + r33 * vz
        
        return np.array([rotated_x, rotated_y, rotated_z])
    
    def _apply_zupt(self, velocity_norm, accel_norm):
        """ZUPT (Zero Velocity Update)を適用（改良版）"""
        if not self.zupt_enabled:
            return False
            
        # 速度と加速度の履歴を更新
        self.velocity_history.append(velocity_norm)
        self.accel_history.append(accel_norm)
        
        # より早い反応のため、10サンプルで判定
        min_samples = 10
        if len(self.velocity_history) >= min_samples:
            # 最近のサンプルのみを使用
            recent_velocity = list(self.velocity_history)[-min_samples:]
            recent_accel = list(self.accel_history)[-min_samples:]
            
            avg_velocity = np.mean(recent_velocity)
            avg_accel = np.mean(recent_accel)
            max_velocity = np.max(recent_velocity)
            max_accel = np.max(recent_accel)
            
            # より厳しい条件：平均と最大値の両方をチェック
            velocity_condition = (avg_velocity < self.zupt_velocity_threshold and 
                                max_velocity < self.zupt_velocity_threshold * 1.5)
            accel_condition = (avg_accel < self.zupt_accel_threshold and 
                             max_accel < self.zupt_accel_threshold * 1.5)
            
            if velocity_condition and accel_condition:
                self.zupt_counter += 1
                # 連続2回に短縮（より早い反応）
                if self.zupt_counter >= 2:
                    logger.debug(f"ZUPT applied: avg_vel={avg_velocity:.4f}, avg_accel={avg_accel:.4f}")
                    return True
            else:
                self.zupt_counter = 0
                
        return False
    
    def _apply_cupt(self):
        """CUPT (Coordinate Update)を適用"""
        if not self.cupt_enabled:
            return False
            
        if self.initial_position is None:
            self.initial_position = self.position.copy()
            return False
            
        # 初期位置からの距離を計算
        distance_from_origin = np.linalg.norm(self.position - self.initial_position)
        
        if distance_from_origin > self.cupt_position_threshold:
            logger.debug(f"CUPT applied: distance={distance_from_origin:.3f}")
            # 位置を初期位置にリセット（ドリフト補正）
            self.position = self.initial_position.copy()
            return True
            
        return False
    
    def _update_trajectory(self, acceleration, timestamp):
        """軌跡を更新（重力補償、バイアス補正、ZUPT/CUPT適用）"""
        if self.prev_timestamp is None:
            self.prev_timestamp = timestamp
            return {
                'position': self.position.copy(),
                'velocity': self.velocity.copy(),
                'acceleration': self.acceleration.copy(),
                'zupt_active': False,
                'cupt_active': False
            }
        
        # 時間差を計算
        dt = timestamp - self.prev_timestamp
        if dt <= 0:  # 時間が逆行または同じ場合はskip
            logger.warning(f"Invalid time delta: {dt}")
            return {
                'position': self.position.copy(),
                'velocity': self.velocity.copy(),
                'acceleration': self.acceleration.copy(),
                'zupt_active': False,
                'cupt_active': False
            }
            
        if dt > 0.5:  # 0.5秒以上の場合は積分をリセット
            logger.warning(f"Large time delta: {dt}s, resetting integration")
            self.velocity = np.array([0.0, 0.0, 0.0])
            self.prev_timestamp = timestamp
            return {
                'position': self.position.copy(),
                'velocity': self.velocity.copy(),
                'acceleration': self.acceleration.copy(),
                'zupt_active': False,
                'cupt_active': False
            }
        
        # バイアス推定（初期化期間）
        if self.bias_estimation_count < self.bias_estimation_samples:
            # 静止状態での加速度バイアスを推定
            if self.bias_estimation_count == 0:
                self.accel_bias = acceleration.copy()
            else:
                # 指数移動平均でバイアスを更新
                alpha = 0.1
                self.accel_bias = (1 - alpha) * self.accel_bias + alpha * acceleration
            
            self.bias_estimation_count += 1
            logger.debug(f"Bias estimation: {self.bias_estimation_count}/{self.bias_estimation_samples}")
            
            # バイアス推定中は軌跡計算しない
            self.prev_timestamp = timestamp
            return {
                'position': self.position.copy(),
                'velocity': self.velocity.copy(),
                'acceleration': self.acceleration.copy(),
                'zupt_active': False,
                'cupt_active': False
            }
        
        # バイアス補正済み加速度
        accel_corrected = acceleration - self.accel_bias
        
        # quaternionを使って加速度をワールド座標系に変換
        accel_world = self._rotate_vector_by_quaternion(accel_corrected, self.q_current)
        
        # 重力ベクトル除去（ワールド座標系）
        # gravity_world = np.array([0.0, 0.0, 9.81])
        gravity_device = np.array([0.0, 0.0, 9.81])
        # デバイス座標系の重力をワールド座標系に変換
        gravity_world = self._rotate_vector_by_quaternion(gravity_device, self.q_current)
        accel_world_no_gravity = accel_world - gravity_world
        
        # ノルムを計算（ZUPT用）
        velocity_norm = np.linalg.norm(self.velocity)
        accel_norm = np.linalg.norm(accel_world_no_gravity)
        
        # ZUPT適用チェック
        zupt_applied = self._apply_zupt(velocity_norm, accel_norm)
        if zupt_applied:
            # 速度をゼロにリセット
            self.velocity = np.array([0.0, 0.0, 0.0])
            # 静止中は加速度も小さくする
            accel_world_no_gravity *= 0.1
        
        # より安定した数値積分（Velocity Verlet法の簡易版）
        # 前の速度を保存
        prev_velocity = self.velocity.copy()
        
        # 速度更新: v = v0 + a*dt
        self.velocity += accel_world_no_gravity * dt
        
        # 位置更新: p = p0 + (v0 + v1)/2 * dt (台形積分)
        avg_velocity = (prev_velocity + self.velocity) / 2.0
        self.position += avg_velocity * dt
        
        # CUPT適用チェック
        cupt_applied = self._apply_cupt()
        
        # 現在の加速度を保存
        self.acceleration = accel_world_no_gravity
        self.prev_timestamp = timestamp
        
        return {
            'position': self.position.copy(),
            'velocity': self.velocity.copy(),
            'acceleration': self.acceleration.copy(),
            'zupt_active': zupt_applied,
            'cupt_active': cupt_applied
        }
    
    def run_realtime(self, enable_ros2=False):
        """リアルタイムIMU処理を開始"""
        logger.info("Starting Madgwick filter with firmware-calibrated IMU data...")
        logger.info("Press Ctrl+C to stop")
        
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
                    
                    # ログ出力（INFO レベルではtrajectory詳細は出力しない）
                    logger.info(f"{sec}.{usec:03d} "
                          f"acc={ax:+.3f},{ay:+.3f},{az:+.3f} "
                          f"gyro={gx:+.3f},{gy:+.3f},{gz:+.3f} "
                          f"q=({q[0]:+.3f},{q[1]:+.3f},{q[2]:+.3f},{q[3]:+.3f}) "
                          f"rpy=({math.degrees(roll):+6.1f},{math.degrees(pitch):+6.1f},{math.degrees(yaw):+6.1f})")
                    
                    # DEBUGレベルでtrajectory情報を出力
                    if hasattr(self, 'position'):
                        logger.debug(f"pos=({self.position[0]:+.3f},{self.position[1]:+.3f},{self.position[2]:+.3f}) "
                               f"vel=({self.velocity[0]:+.3f},{self.velocity[1]:+.3f},{self.velocity[2]:+.3f})")
                    
        except KeyboardInterrupt:
            logger.info("Stopping IMU processing...")
        finally:
            if ros2_bridge:
                ros2_bridge.stop_spinning()


def parse_arguments():
    """コマンドライン引数をパース"""
    parser = argparse.ArgumentParser(
        description='Spresense IMU Trajectory Tracker with Madgwick Filter',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python 2_madgwick_trajectory.py                    # Basic usage
  python 2_madgwick_trajectory.py --ros2             # Enable ROS2 publishing
  python 2_madgwick_trajectory.py --no-zupt          # Disable ZUPT
  python 2_madgwick_trajectory.py --no-cupt          # Disable CUPT
  python 2_madgwick_trajectory.py --ros2 --no-zupt   # ROS2 with ZUPT disabled
  python 2_madgwick_trajectory.py --frequency 100    # Custom frequency

Environment Variables:
  LOG_LEVEL=DEBUG|INFO|WARNING|ERROR (default: INFO)
  ZUPT_VELOCITY_THRESHOLD=0.1 (default: 0.1 m/s)
  ZUPT_ACCEL_THRESHOLD=0.5 (default: 0.5 m/s²)
  CUPT_POSITION_THRESHOLD=10.0 (default: 10.0 m)
        """
    )
    
    # ROS2 options
    parser.add_argument('--ros2', action='store_true',
                        help='Enable ROS2 publishing (default: False)')
    parser.add_argument('--no-ros2', dest='ros2', action='store_false',
                        help='Disable ROS2 publishing')
    
    # ZUPT/CUPT options
    parser.add_argument('--zupt', action='store_true', default=ZUPT_ENABLED,
                        help='Enable ZUPT (Zero Velocity Update)')
    parser.add_argument('--no-zupt', dest='zupt', action='store_false',
                        help='Disable ZUPT (Zero Velocity Update)')
    parser.add_argument('--cupt', action='store_true', default=CUPT_ENABLED,
                        help='Enable CUPT (Coordinate Update)')
    parser.add_argument('--no-cupt', dest='cupt', action='store_false',
                        help='Disable CUPT (Coordinate Update)')
    
    # Frequency setting
    parser.add_argument('--frequency', type=float, default=120.0,
                        help='IMU sampling frequency in Hz (default: 60.0)')
    
    # Threshold settings
    parser.add_argument('--zupt-vel-threshold', type=float, default=ZUPT_VELOCITY_THRESHOLD,
                        help=f'ZUPT velocity threshold (default: {ZUPT_VELOCITY_THRESHOLD})')
    parser.add_argument('--zupt-accel-threshold', type=float, default=ZUPT_ACCEL_THRESHOLD,
                        help=f'ZUPT acceleration threshold (default: {ZUPT_ACCEL_THRESHOLD})')
    parser.add_argument('--cupt-pos-threshold', type=float, default=CUPT_POSITION_THRESHOLD,
                        help=f'CUPT position threshold (default: {CUPT_POSITION_THRESHOLD})')
    
    # Default ROS2 to environment variable if not specified
    parser.set_defaults(ros2=os.getenv('ENABLE_ROS2', 'False').lower() == 'true')
    
    return parser.parse_args()


if __name__ == "__main__":
    # コマンドライン引数をパース
    args = parse_arguments()
    
    # ログレベルの使用例
    logger.info(f"Log level set to: {LOG_LEVEL}")
    logger.debug("Debug messages will be shown if log level is DEBUG")
    
    # 設定の表示
    logger.info(f"Configuration:")
    logger.info(f"  Frequency: {args.frequency} Hz")
    logger.info(f"  ROS2 publishing: {args.ros2}")
    logger.info(f"  ZUPT enabled: {args.zupt}")
    logger.info(f"  CUPT enabled: {args.cupt}")
    if args.zupt:
        logger.info(f"  ZUPT velocity threshold: {args.zupt_vel_threshold} m/s")
        logger.info(f"  ZUPT acceleration threshold: {args.zupt_accel_threshold} m/s²")
    if args.cupt:
        logger.info(f"  CUPT position threshold: {args.cupt_pos_threshold} m")
    
    # プロセッサーを初期化
    processor = SpresenseIMUProcessor(
        frequency=args.frequency,
        zupt_enabled=args.zupt,
        cupt_enabled=args.cupt
    )
    
    # 閾値を設定（コマンドライン引数から）
    processor.zupt_velocity_threshold = args.zupt_vel_threshold
    processor.zupt_accel_threshold = args.zupt_accel_threshold
    processor.cupt_position_threshold = args.cupt_pos_threshold
    
    # 実行
    processor.run_realtime(enable_ros2=args.ros2)
