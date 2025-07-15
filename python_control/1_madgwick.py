import numpy as np
from ahrs.filters import Madgwick
from utils.imu import IMUReader
import math
import threading
import queue
import time

import dash
from dash import dcc, html, Input, Output
import plotly.graph_objects as go

# ========= 環境設定 =========================================
PORT     = "/dev/cu.usbserial-1140" 
BAUDRATE = 115_200

class SpresenseIMUProcessor:
    """IMU処理専用クラス - ビジュアライゼーションとは完全に分離"""
    
    def __init__(self, frequency=1920.0, gain=0.1):  # gainを大幅に増加
        self.madgwick = Madgwick(frequency=frequency, gain=gain)
        self.q_current = np.array([1.0, 0.0, 0.0, 0.0])
        self.imu_reader = IMUReader(port=PORT, baudrate=BAUDRATE)
        self.static_threshold = 0.001  # より小さい閾値に変更
        
        # コールバック関数のリスト
        self.quaternion_callbacks = []
        
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
    
    def process_imu_data(self, sec, msec, ax, ay, az, gx, gy, gz):
        """IMUデータを処理してquaternionを更新"""
        # Debug: Print raw gyro values to check for non-zero data
        if abs(gx) > 0.01 or abs(gy) > 0.01 or abs(gz) > 0.01:
            print(f"DEBUG: Non-zero gyro detected: gx={gx:.6f}, gy={gy:.6f}, gz={gz:.6f}")
        
        acc_ms2 = np.array([ax, ay, az])
        gyro_rads = np.array([gx, gy, gz]) * np.pi / 180.0
        
        # Debug: Check if conversion causes zeros
        if np.any(np.abs(gyro_rads) > 0.001):
            print(f"DEBUG: gyro_rads = {gyro_rads}")
        
        # ジャイロマグニチュードを計算
        gyro_magnitude = np.linalg.norm(gyro_rads)
        print(f"DEBUG: gyro_magnitude = {gyro_magnitude:.6f}, threshold = {self.static_threshold}")
        
        # 静的検出の条件を緩和し、動的なgain調整
        if gyro_magnitude < self.static_threshold:
            # 静的条件でも適度なgainを維持
            temp_gain = self.madgwick.gain
            self.madgwick.gain = 0.01  # 0.0005から0.01に増加
            print("DEBUG: Using static gain (0.01)")
            self.q_current = self.madgwick.updateIMU(
                self.q_current, gyr=gyro_rads, acc=acc_ms2
            )
            self.madgwick.gain = temp_gain  # Restore original gain
        else:
            print(f"DEBUG: Using dynamic gain ({self.madgwick.gain})")
            self.q_current = self.madgwick.updateIMU(
                self.q_current, gyr=gyro_rads, acc=acc_ms2
            )
        
        # コールバックに通知
        timestamp = f"{sec}.{msec:03d}"
        raw_data = {
            'acc': [ax, ay, az],
            'gyro': [gx, gy, gz],
            'euler': self.quaternion_to_euler(self.q_current)
        }
        self._notify_quaternion_update(self.q_current, timestamp, raw_data)
        
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
    
    def run_realtime(self):
        """リアルタイムIMU処理を開始"""
        print("Starting Madgwick filter with firmware-calibrated IMU data...")
        print("Press Ctrl+C to stop")
        
        try:
            with self.imu_reader as reader:
                for sec, msec, ax, ay, az, gx, gy, gz in reader.stream_data():
                    q = self.process_imu_data(sec, msec, ax, ay, az, gx, gy, gz)
                    roll, pitch, yaw = self.quaternion_to_euler(q)
                    
                    print(f"{sec}.{msec:03d} "
                          f"acc={ax:+.3f},{ay:+.3f},{az:+.3f} "
                          f"gyro={gx:+.3f},{gy:+.3f},{gz:+.3f} "
                          f"q=({q[0]:+.3f},{q[1]:+.3f},{q[2]:+.3f},{q[3]:+.3f}) "
                          f"rpy=({math.degrees(roll):+6.1f},{math.degrees(pitch):+6.1f},{math.degrees(yaw):+6.1f})")
        except KeyboardInterrupt:
            print("\nStopping IMU processing...")


class PlotlyIMUVisualizer:
    """Plotly Dash専用ビジュアライゼーションクラス"""
    
    def __init__(self, port=8050):
        self.app = dash.Dash(__name__)
        self.port = port
        self.current_q = np.array([1.0, 0.0, 0.0, 0.0])
        self.current_timestamp = "0.000"
        self.current_raw_data = {'acc': [0, 0, 0], 'gyro': [0, 0, 0], 'euler': [0, 0, 0]}
        
        # データ更新用のキュー（最新の1つのみ保持）
        self.data_queue = queue.Queue(maxsize=1)
        
        self._setup_layout()
        self._setup_callbacks()
        
    def _setup_layout(self):
        """Dashアプリのレイアウトを設定"""
        self.app.layout = html.Div([
            html.H1("IMU Orientation Visualizer", 
                   style={'textAlign': 'center', 'marginBottom': 20}),
            
            html.Div([
                html.Div([
                    html.H3("Raw Data"),
                    html.Div(id='raw-data-display')
                ], style={'width': '30%', 'display': 'inline-block', 'verticalAlign': 'top'}),
                
                html.Div([
                    dcc.Graph(id='3d-orientation', style={'height': '600px'})
                ], style={'width': '70%', 'display': 'inline-block'})
            ]),
            
            dcc.Interval(
                id='interval-component',
                interval=50,  # 20Hz update (50ms)
                n_intervals=0
            )
        ])
    
    def _setup_callbacks(self):
        """Dashコールバックを設定"""
        @self.app.callback(
            [Output('3d-orientation', 'figure'),
             Output('raw-data-display', 'children')],
            [Input('interval-component', 'n_intervals')]
        )
        def update_display(n):
            # 最新データを取得
            try:
                while not self.data_queue.empty():
                    data = self.data_queue.get_nowait()
                    self.current_q = data['quaternion']
                    self.current_timestamp = data['timestamp']
                    self.current_raw_data = data['raw_data']
            except queue.Empty:
                pass
            
            return self._create_3d_figure(), self._create_raw_data_display()
    
    def _quaternion_to_rotation_matrix(self, q):
        """quaternionを回転行列に変換"""
        w, x, y, z = q
        R = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
        ])
        return R
    
    def _create_3d_figure(self):
        """3D座標軸の図を作成"""
        R = self._quaternion_to_rotation_matrix(self.current_q)
        origin = [0, 0, 0]
        
        # 座標軸ベクトル
        x_axis = R[:, 0] * 0.8
        y_axis = R[:, 1] * 0.8
        z_axis = R[:, 2] * 0.8
        
        fig = go.Figure()
        
        # X軸 (赤)
        fig.add_trace(go.Scatter3d(
            x=[origin[0], origin[0] + x_axis[0]],
            y=[origin[1], origin[1] + x_axis[1]],
            z=[origin[2], origin[2] + x_axis[2]],
            mode='lines+markers',
            line=dict(color='red', width=10),
            marker=dict(size=8, color='red'),
            name='X-axis',
            showlegend=True
        ))
        
        # Y軸 (緑)
        fig.add_trace(go.Scatter3d(
            x=[origin[0], origin[0] + y_axis[0]],
            y=[origin[1], origin[1] + y_axis[1]],
            z=[origin[2], origin[2] + y_axis[2]],
            mode='lines+markers',
            line=dict(color='green', width=10),
            marker=dict(size=8, color='green'),
            name='Y-axis',
            showlegend=True
        ))
        
        # Z軸 (青)
        fig.add_trace(go.Scatter3d(
            x=[origin[0], origin[0] + z_axis[0]],
            y=[origin[1], origin[1] + z_axis[1]],
            z=[origin[2], origin[2] + z_axis[2]],
            mode='lines+markers',
            line=dict(color='blue', width=10),
            marker=dict(size=8, color='blue'),
            name='Z-axis',
            showlegend=True
        ))
        
        fig.update_layout(
            title=f'IMU Orientation - {self.current_timestamp}',
            scene=dict(
                xaxis=dict(range=[-1, 1], title='X'),
                yaxis=dict(range=[-1, 1], title='Y'),
                zaxis=dict(range=[-1, 1], title='Z'),
                aspectmode='cube',
                camera=dict(
                    eye=dict(x=1.5, y=1.5, z=1.5)
                )
            ),
            showlegend=True,
            margin=dict(l=0, r=0, t=40, b=0)
        )
        
        return fig
    
    def _create_raw_data_display(self):
        """生データ表示を作成"""
        ax, ay, az = self.current_raw_data['acc']
        gx, gy, gz = self.current_raw_data['gyro']
        roll, pitch, yaw = self.current_raw_data['euler']
        
        return html.Div([
            html.P(f"Timestamp: {self.current_timestamp}"),
            html.Hr(),
            html.H4("Accelerometer (m/s²)"),
            html.P(f"X: {ax:+.3f}"),
            html.P(f"Y: {ay:+.3f}"),
            html.P(f"Z: {az:+.3f}"),
            html.Hr(),
            html.H4("Gyroscope (°/s)"),
            html.P(f"X: {gx:+.3f}"),
            html.P(f"Y: {gy:+.3f}"),
            html.P(f"Z: {gz:+.3f}"),
            html.Hr(),
            html.H4("Euler Angles (°)"),
            html.P(f"Roll: {math.degrees(roll):+6.1f}"),
            html.P(f"Pitch: {math.degrees(pitch):+6.1f}"),
            html.P(f"Yaw: {math.degrees(yaw):+6.1f}"),
            html.Hr(),
            html.H4("Quaternion"),
            html.P(f"w: {self.current_q[0]:+.3f}"),
            html.P(f"x: {self.current_q[1]:+.3f}"),
            html.P(f"y: {self.current_q[2]:+.3f}"),
            html.P(f"z: {self.current_q[3]:+.3f}")
        ])
    
    def update_from_imu(self, quaternion, timestamp, raw_data):
        """IMU処理クラスから呼び出される更新メソッド"""
        data = {
            'quaternion': quaternion.copy(),
            'timestamp': timestamp,
            'raw_data': raw_data.copy()
        }
        
        try:
            # 最新データのみ保持（古いデータは破棄）
            while not self.data_queue.empty():
                self.data_queue.get_nowait()
            self.data_queue.put_nowait(data)
        except queue.Full:
            pass  # キューがフルの場合は無視
    
    def start_server(self, debug=False):
        """Webサーバーを別スレッドで開始"""
        def run_server():
            try:
                print(f"Starting Dash server on port {self.port}...")
                self.app.run(
                    debug=debug, 
                    port=self.port, 
                    host='127.0.0.1',  # 明示的にhostを指定
                    use_reloader=False,
                    dev_tools_hot_reload=False
                )
            except Exception as e:
                print(f"Server startup error: {e}")
                # 別のポートを試す
                alternative_port = self.port + 1
                print(f"Trying alternative port {alternative_port}...")
                try:
                    self.port = alternative_port
                    self.app.run_server(
                        debug=debug, 
                        port=alternative_port, 
                        host='127.0.0.1',
                        use_reloader=False,
                        dev_tools_hot_reload=False
                    )
                except Exception as e2:
                    print(f"Alternative port also failed: {e2}")
        
        server_thread = threading.Thread(target=run_server)
        server_thread.daemon = True
        server_thread.start()
        print(f"Plotly Dash server starting at http://127.0.0.1:{self.port}")
        return server_thread


def main():
    """メイン実行関数"""
    print("Starting IMU Visualizer...")
    
    # ビジュアライザーを作成・開始
    visualizer = PlotlyIMUVisualizer(port=8050)
    server_thread = visualizer.start_server(debug=True)  # デバッグモードを有効
    
    # サーバーが起動するまで待機
    print("Waiting for server to start...")
    time.sleep(3)
    
    # サーバーが動作しているか確認
    import urllib.request
    try:
        response = urllib.request.urlopen(f'http://127.0.0.1:{visualizer.port}', timeout=5)
        print(f"✓ Server is running! Access: http://127.0.0.1:{visualizer.port}")
    except Exception as e:
        print(f"✗ Server check failed: {e}")
        print("Try accessing manually or check console for errors")
    
    # IMU処理クラスを作成
    imu_processor = SpresenseIMUProcessor()
    
    # IMU処理クラスにビジュアライザーのコールバックを登録
    imu_processor.add_quaternion_callback(visualizer.update_from_imu)
    
    print("Starting IMU processing...")
    print(f"Access the visualizer at: http://127.0.0.1:{visualizer.port}")
    
    # IMU処理を開始（メインスレッドで実行）
    try:
        imu_processor.run_realtime()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        # コールバックを削除（クリーンアップ）
        imu_processor.remove_quaternion_callback(visualizer.update_from_imu)


if __name__ == "__main__":
    main()