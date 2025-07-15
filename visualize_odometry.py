#!/usr/bin/env python3
"""
改良版IMU位置推定システム
- クォータニオンベースの姿勢推定
- ゼロ速度補正（ZUPT）
- より正確な重力補正
"""

import struct
import serial
import crc8
import signal
import sys
import atexit
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import threading
import time
from collections import deque
import math

# ========= 環境設定 =========================================
PORT     = "/dev/cu.usbserial-1130"
BAUDRATE = 115_200
TIMEOUT  = 1.0

# 物理定数
GRAVITY_AMOUNT = 9.80665
MEASUREMENT_FREQUENCY = 960  # Hz

# ノイズパラメータ
GYRO_NOISE_DENSITY = 1.0e-3 * np.pi / 180
ACCEL_NOISE_DENSITY = 14.0e-6 * GRAVITY_AMOUNT
ACCEL_BIAS_DRIFT = 4.43e-6 * GRAVITY_AMOUNT
GYRO_BIAS_DRIFT = 0.39 * np.pi / 180

# 3D表示設定
MAX_TRAJECTORY_POINTS = 1000
PLOT_UPDATE_INTERVAL = 0.05

# ZUPT設定
ZUPT_VELOCITY_THRESHOLD = 0.1  # m/s
ZUPT_ACCELERATION_THRESHOLD = 0.2  # m/s²
ZUPT_ANGULAR_THRESHOLD = 0.1  # rad/s
ZUPT_DURATION_THRESHOLD = 0.5  # s
# ===========================================================

STRUCT_PAYLOAD = struct.Struct("<cII6fB")
FRAME_SIZE = 36

class QuaternionINS:
    """クォータニオンベースの慣性航法システム"""
    
    def __init__(self):
        # 姿勢（クォータニオン: w, x, y, z）
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        
        # 位置・速度・加速度（世界座標系）
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.acceleration = np.array([0.0, 0.0, 0.0])
        
        # 重力ベクトル（世界座標系）
        self.gravity = np.array([0.0, 0.0, -GRAVITY_AMOUNT])
        
        # バイアス推定
        self.accel_bias = np.array([0.0, 0.0, 0.0])
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        
        # 校正用
        self.calibration_samples = []
        self.is_calibrated = False
        self.calibration_count = 200
        
        # ZUPT関連
        self.zupt_detector = ZUPTDetector()
        
        # 履歴
        self.trajectory = deque(maxlen=MAX_TRAJECTORY_POINTS)
        self.trajectory_lock = threading.Lock()
        
        # 時刻管理
        self.last_time = None
        self.sample_count = 0
        
    def quaternion_mult(self, q1, q2):
        """クォータニオンの掛け算"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])
    
    def quaternion_conjugate(self, q):
        """クォータニオンの共役"""
        return np.array([q[0], -q[1], -q[2], -q[3]])
    
    def rotate_vector(self, v, q):
        """クォータニオンでベクトルを回転"""
        # v を純クォータニオンに変換
        qv = np.array([0.0, v[0], v[1], v[2]])
        
        # q * qv * q^(-1)
        q_conj = self.quaternion_conjugate(q)
        temp = self.quaternion_mult(q, qv)
        result = self.quaternion_mult(temp, q_conj)
        
        return result[1:4]  # ベクトル部分を返す
    
    def quaternion_derivative(self, q, omega):
        """クォータニオンの時間微分"""
        w, x, y, z = q
        wx, wy, wz = omega
        
        return 0.5 * np.array([
            -x*wx - y*wy - z*wz,
             w*wx + y*wz - z*wy,
             w*wy - x*wz + z*wx,
             w*wz + x*wy - y*wx
        ])
    
    def rk4_quaternion_update(self, q, omega, dt):
        """RK4法でクォータニオンを更新"""
        k1 = self.quaternion_derivative(q, omega)
        k2 = self.quaternion_derivative(q + dt/2 * k1, omega)
        k3 = self.quaternion_derivative(q + dt/2 * k2, omega)
        k4 = self.quaternion_derivative(q + dt * k3, omega)
        
        q_new = q + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
        
        # 正規化
        norm = np.linalg.norm(q_new)
        if norm > 0:
            q_new = q_new / norm
        
        return q_new
    
    def calibrate(self, acc_data, gyro_data):
        """センサーの校正"""
        if not self.is_calibrated:
            self.calibration_samples.append({
                'acc': acc_data.copy(),
                'gyro': gyro_data.copy()
            })
            
            if len(self.calibration_samples) >= self.calibration_count:
                # バイアス推定
                acc_samples = np.array([s['acc'] for s in self.calibration_samples])
                gyro_samples = np.array([s['gyro'] for s in self.calibration_samples])
                
                self.accel_bias = np.mean(acc_samples, axis=0)
                self.gyro_bias = np.mean(gyro_samples, axis=0)
                
                # 重力方向を推定
                gravity_magnitude = np.linalg.norm(self.accel_bias)
                if gravity_magnitude > 0:
                    self.accel_bias = self.accel_bias - self.accel_bias * (GRAVITY_AMOUNT / gravity_magnitude)
                
                self.is_calibrated = True
                print(f"校正完了:")
                print(f"  加速度バイアス: {self.accel_bias}")
                print(f"  ジャイロバイアス: {self.gyro_bias}")
                print(f"  重力の大きさ: {gravity_magnitude:.3f} m/s²")
                
                return True
            else:
                print(f"校正中... {len(self.calibration_samples)}/{self.calibration_count}")
                return False
        
        return True
    
    def update(self, timestamp, acc_data, gyro_data):
        """IMUデータを更新"""
        if not self.calibrate(acc_data, gyro_data):
            return
        
        if self.last_time is None:
            self.last_time = timestamp
            return
        
        dt = timestamp - self.last_time
        if dt <= 0 or dt > 0.1:
            self.last_time = timestamp
            return
        
        # バイアス補正
        acc_corrected = acc_data - self.accel_bias
        gyro_corrected = gyro_data - self.gyro_bias
        
        # 姿勢更新（ジャイロスコープ）
        self.q = self.rk4_quaternion_update(self.q, gyro_corrected, dt)
        
        # 加速度をボディ座標系から世界座標系に変換
        acc_world = self.rotate_vector(acc_corrected, self.q)
        
        # 重力補正
        linear_acc = acc_world - self.gravity
        
        # ZUPT検出
        is_stationary = self.zupt_detector.detect(
            linear_acc, self.velocity, gyro_corrected, dt
        )
        
        if is_stationary:
            # 静止状態: 速度をリセット
            self.velocity = np.array([0.0, 0.0, 0.0])
            print("ZUPT適用: 速度リセット")
        else:
            # 速度更新
            self.velocity += linear_acc * dt
        
        # 位置更新
        self.position += self.velocity * dt
        
        # 軌跡に追加
        if self.sample_count % 10 == 0:
            with self.trajectory_lock:
                self.trajectory.append(self.position.copy())
        
        self.sample_count += 1
        self.last_time = timestamp
        self.acceleration = linear_acc

class ZUPTDetector:
    """ゼロ速度更新検出器"""
    
    def __init__(self):
        self.stationary_time = 0.0
        self.acc_buffer = deque(maxlen=20)
        self.gyro_buffer = deque(maxlen=20)
        
    def detect(self, acceleration, velocity, angular_velocity, dt):
        """静止状態を検出"""
        self.acc_buffer.append(acceleration)
        self.gyro_buffer.append(angular_velocity)
        
        # 十分なサンプルが溜まるまで待つ
        if len(self.acc_buffer) < 10:
            return False
        
        # 加速度の分散をチェック
        acc_var = np.var(np.array(self.acc_buffer), axis=0)
        acc_still = np.all(acc_var < ZUPT_ACCELERATION_THRESHOLD**2)
        
        # 角速度の分散をチェック
        gyro_var = np.var(np.array(self.gyro_buffer), axis=0)
        gyro_still = np.all(gyro_var < ZUPT_ANGULAR_THRESHOLD**2)
        
        # 速度の大きさをチェック
        velocity_small = np.linalg.norm(velocity) < ZUPT_VELOCITY_THRESHOLD
        
        if acc_still and gyro_still and velocity_small:
            self.stationary_time += dt
            return self.stationary_time > ZUPT_DURATION_THRESHOLD
        else:
            self.stationary_time = 0.0
            return False

class IMUVisualizer:
    """3D可視化クラス"""
    def __init__(self, ins):
        self.ins = ins
        self.setup_plot()
        
    def setup_plot(self):
        """プロットを設定"""
        plt.ion()
        self.fig = plt.figure(figsize=(15, 10))
        
        # 3D軌跡プロット
        self.ax3d = self.fig.add_subplot(221, projection='3d')
        self.trajectory_line, = self.ax3d.plot([], [], [], 'b-', alpha=0.7, linewidth=2)
        self.current_point, = self.ax3d.plot([], [], [], 'ro', markersize=10)
        
        self.ax3d.set_xlabel('X [m]')
        self.ax3d.set_ylabel('Y [m]')
        self.ax3d.set_zlabel('Z [m]')
        self.ax3d.set_title('3D Trajectory (with ZUPT)')
        self.ax3d.grid(True)
        
        # 姿勢表示
        self.ax_attitude = self.fig.add_subplot(222)
        self.ax_attitude.set_xlabel('Time [s]')
        self.ax_attitude.set_ylabel('Quaternion')
        self.ax_attitude.set_title('Attitude (Quaternion)')
        self.ax_attitude.grid(True)
        
        # 速度履歴
        self.ax_velocity = self.fig.add_subplot(223)
        self.ax_velocity.set_xlabel('Time [s]')
        self.ax_velocity.set_ylabel('Velocity [m/s]')
        self.ax_velocity.set_title('Velocity')
        self.ax_velocity.grid(True)
        
        # 位置履歴
        self.ax_position = self.fig.add_subplot(224)
        self.ax_position.set_xlabel('Time [s]')
        self.ax_position.set_ylabel('Position [m]')
        self.ax_position.set_title('Position')
        self.ax_position.grid(True)
        
        plt.tight_layout()
        
        # 履歴用
        self.time_history = deque(maxlen=200)
        self.q_history = deque(maxlen=200)
        self.vel_history = deque(maxlen=200)
        self.pos_history = deque(maxlen=200)
        
    def update_plot(self):
        """プロットを更新"""
        with self.ins.trajectory_lock:
            if len(self.ins.trajectory) < 2:
                return
            trajectory = np.array(list(self.ins.trajectory))
        
        # 3D軌跡更新
        self.trajectory_line.set_data(trajectory[:, 0], trajectory[:, 1])
        self.trajectory_line.set_3d_properties(trajectory[:, 2])
        
        current_pos = trajectory[-1]
        self.current_point.set_data([current_pos[0]], [current_pos[1]])
        self.current_point.set_3d_properties([current_pos[2]])
        
        # 軸範囲調整
        margin = 0.5
        ranges = []
        for i in range(3):
            data_range = trajectory[:, i]
            if len(data_range) > 0:
                min_val, max_val = data_range.min(), data_range.max()
                center = (min_val + max_val) / 2
                span = max(max_val - min_val, 0.5)
                ranges.append([center - span/2 - margin, center + span/2 + margin])
            else:
                ranges.append([-margin, margin])
        
        self.ax3d.set_xlim(ranges[0])
        self.ax3d.set_ylim(ranges[1])
        self.ax3d.set_zlim(ranges[2])
        
        # 履歴更新
        current_time = time.time()
        self.time_history.append(current_time)
        self.q_history.append(self.ins.q.copy())
        self.vel_history.append(self.ins.velocity.copy())
        self.pos_history.append(self.ins.position.copy())
        
        if len(self.time_history) > 1:
            time_array = np.array(self.time_history)
            time_array = time_array - time_array[0]
            
            # 姿勢プロット
            self.ax_attitude.clear()
            q_array = np.array(self.q_history)
            self.ax_attitude.plot(time_array, q_array[:, 0], 'r-', label='w', alpha=0.7)
            self.ax_attitude.plot(time_array, q_array[:, 1], 'g-', label='x', alpha=0.7)
            self.ax_attitude.plot(time_array, q_array[:, 2], 'b-', label='y', alpha=0.7)
            self.ax_attitude.plot(time_array, q_array[:, 3], 'm-', label='z', alpha=0.7)
            self.ax_attitude.set_xlabel('Time [s]')
            self.ax_attitude.set_ylabel('Quaternion')
            self.ax_attitude.set_title('Attitude (Quaternion)')
            self.ax_attitude.legend()
            self.ax_attitude.grid(True)
            
            # 速度プロット
            self.ax_velocity.clear()
            vel_array = np.array(self.vel_history)
            self.ax_velocity.plot(time_array, vel_array[:, 0], 'r-', label='Vx', alpha=0.7)
            self.ax_velocity.plot(time_array, vel_array[:, 1], 'g-', label='Vy', alpha=0.7)
            self.ax_velocity.plot(time_array, vel_array[:, 2], 'b-', label='Vz', alpha=0.7)
            self.ax_velocity.set_xlabel('Time [s]')
            self.ax_velocity.set_ylabel('Velocity [m/s]')
            self.ax_velocity.set_title('Velocity')
            self.ax_velocity.legend()
            self.ax_velocity.grid(True)
            
            # 位置プロット
            self.ax_position.clear()
            pos_array = np.array(self.pos_history)
            self.ax_position.plot(time_array, pos_array[:, 0], 'r-', label='X', alpha=0.7)
            self.ax_position.plot(time_array, pos_array[:, 1], 'g-', label='Y', alpha=0.7)
            self.ax_position.plot(time_array, pos_array[:, 2], 'b-', label='Z', alpha=0.7)
            self.ax_position.set_xlabel('Time [s]')
            self.ax_position.set_ylabel('Position [m]')
            self.ax_position.set_title('Position')
            self.ax_position.legend()
            self.ax_position.grid(True)
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def crc8_maxim(data: bytes) -> int:
    """Dallas/Maxim CRC-8"""
    h = crc8.crc8()
    h.update(data)
    return h.digest()[0]

# Global variables
_serial_port = None
_ins = None
_visualizer = None
_running = False

def cleanup_serial():
    """シリアルポートをクリーンアップ"""
    global _serial_port, _running
    _running = False
    if _serial_port and _serial_port.is_open:
        print("\nシリアルポートを閉じています...")
        _serial_port.close()
        _serial_port = None

def signal_handler(signum, frame):
    """シグナルハンドラ"""
    print(f"\nシグナル {signum} を受信、シャットダウン中...")
    cleanup_serial()
    sys.exit(0)

def main():
    global _serial_port, _ins, _visualizer, _running
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    atexit.register(cleanup_serial)
    
    print(f"ポート {PORT} を {BAUDRATE} bps で開いています")
    print("センサーを静止状態に保って校正を開始してください...")
    
    _ins = QuaternionINS()
    _running = True
    
    try:
        _serial_port = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
        _serial_port.reset_input_buffer()
        
        _visualizer = IMUVisualizer(_ins)
        last_plot_update = time.time()
        
        while _running:
            buf = _serial_port.read(FRAME_SIZE)
            if len(buf) != FRAME_SIZE:
                continue

            payload, crlf = buf[:-2], buf[-2:]
            if crlf != b"\r\n":
                _serial_port.read(1)
                continue

            try:
                header, sec, msec, *vals, crc_recv = STRUCT_PAYLOAD.unpack(payload)
            except struct.error:
                continue

            if header != b"X":
                continue

            crc_calc = crc8_maxim(payload[:-1])
            if crc_calc != crc_recv:
                continue

            ax, ay, az, gx, gy, gz = vals
            timestamp = sec + msec / 1000.0
            
            acc_data = np.array([ax, ay, az])
            gyro_data = np.array([gx, gy, gz])
            
            _ins.update(timestamp, acc_data, gyro_data)
            
            # プロット更新
            current_time = time.time()
            if current_time - last_plot_update > PLOT_UPDATE_INTERVAL:
                if _ins.is_calibrated:
                    _visualizer.update_plot()
                last_plot_update = current_time
            
            # 位置情報出力
            if _ins.is_calibrated and _ins.sample_count % 50 == 0:
                pos = _ins.position
                vel = _ins.velocity
                print(f"位置: ({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}) m  "
                      f"速度: ({vel[0]:+.3f}, {vel[1]:+.3f}, {vel[2]:+.3f}) m/s")
                  
    except serial.SerialException as e:
        print(f"シリアルエラー: {e}")
        cleanup_serial()
        sys.exit(1)
    except Exception as e:
        print(f"予期しないエラー: {e}")
        cleanup_serial()
        sys.exit(1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nユーザーによって停止されました")
        cleanup_serial()
    except Exception as e:
        print(f"エラー: {e}")
        cleanup_serial()
        sys.exit(1)