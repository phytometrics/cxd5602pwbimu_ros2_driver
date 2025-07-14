#!/usr/bin/env python3
"""
相対距離ベースの軌跡表示 - 積分エラーを最小化
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import signal
import sys
import atexit

class RelativeTrajectoryNode(Node):
    def __init__(self):
        super().__init__('relative_trajectory_node')
        
        # Subscribe to filtered IMU data
        self.imu_subscriber = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        
        # Publishers
        self.trajectory_publisher = self.create_publisher(Path, '/robot/relative_trajectory', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/robot/relative_pose', 10)
        
        # Initialize path
        self.path = Path()
        self.path.header.frame_id = "odom"
        
        # Relative position tracking
        self.relative_position = np.array([0.0, 0.0, 0.0])
        self.last_significant_position = np.array([0.0, 0.0, 0.0])
        
        # Movement detection
        self.movement_segments = []
        self.current_segment_start = None
        self.movement_threshold = 0.1
        self.segment_distance = 0.0
        
        # Parameters
        self.declare_parameter('segment_length', 0.5)  # セグメント長
        self.declare_parameter('max_path_length', 1000)
        
        self.segment_length = self.get_parameter('segment_length').value
        self.max_path_length = self.get_parameter('max_path_length').value
        
        self.get_logger().info('Relative Trajectory Node started')
    
    def imu_callback(self, msg):
        # 加速度の大きさで移動を検出
        linear_accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        accel_magnitude = np.linalg.norm(linear_accel)
        
        # 相対的な移動セグメントを記録
        if accel_magnitude > self.movement_threshold:
            if self.current_segment_start is None:
                self.current_segment_start = self.relative_position.copy()
                self.get_logger().info("New movement segment started")
            
            # 概算の移動距離を追加（加速度から推定）
            estimated_movement = accel_magnitude * 0.001  # 簡略化
            
            # 移動方向を姿勢から推定
            orientation = msg.orientation
            
            # Z軸回転（ヨー）を取得
            yaw = 2 * math.atan2(orientation.z, orientation.w)
            
            # 移動方向を推定（前方向への移動と仮定）
            movement_x = estimated_movement * math.cos(yaw)
            movement_y = estimated_movement * math.sin(yaw)
            
            self.relative_position[0] += movement_x
            self.relative_position[1] += movement_y
            
            self.segment_distance += estimated_movement
            
        else:
            # 静止状態 - セグメントを完了
            if self.current_segment_start is not None:
                if self.segment_distance > self.segment_length:
                    # セグメントを保存
                    segment_end = self.relative_position.copy()
                    self.movement_segments.append((self.current_segment_start, segment_end))
                    self.last_significant_position = segment_end.copy()
                    
                    self.get_logger().info(f"Movement segment completed: distance={self.segment_distance:.3f}")
                
                self.current_segment_start = None
                self.segment_distance = 0.0
        
        # 軌跡を再構築
        self.reconstruct_trajectory(msg.header.stamp)
    
    def reconstruct_trajectory(self, timestamp):
        """セグメントから軌跡を再構築"""
        # 現在の位置を使用
        current_position = self.relative_position.copy()
        
        # PoseStampedを作成
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = timestamp
        pose_stamped.header.frame_id = "odom"
        pose_stamped.pose.position.x = current_position[0]
        pose_stamped.pose.position.y = current_position[1]
        pose_stamped.pose.position.z = current_position[2]
        pose_stamped.pose.orientation.w = 1.0  # 単位クォータニオン
        
        # Publish current pose
        self.pose_publisher.publish(pose_stamped)
        
        # 軌跡に追加（距離ベースフィルタリング）
        if len(self.path.poses) == 0:
            self.path.poses.append(pose_stamped)
        else:
            last_pose = self.path.poses[-1]
            distance = math.sqrt(
                (pose_stamped.pose.position.x - last_pose.pose.position.x)**2 +
                (pose_stamped.pose.position.y - last_pose.pose.position.y)**2 +
                (pose_stamped.pose.position.z - last_pose.pose.position.z)**2
            )
            
            if distance > 0.01:  # 1cm以上の移動で記録
                self.path.poses.append(pose_stamped)
        
        # パス長制限
        if len(self.path.poses) > self.max_path_length:
            self.path.poses.pop(0)
        
        # 軌跡を公開
        self.path.header.stamp = timestamp
        self.trajectory_publisher.publish(self.path)

# Global variable for node cleanup
_node_instance = None

def cleanup_node():
    global _node_instance
    if _node_instance is not None:
        _node_instance.destroy_node()
        _node_instance = None
    if rclpy.ok():
        rclcpp.shutdown()

def signal_handler(signum, frame):
    cleanup_node()
    sys.exit(0)

def main(args=None):
    global _node_instance
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    atexit.register(cleanup_node)
    
    try:
        rclpy.init(args=args)
        _node_instance = RelativeTrajectoryNode()
        rclpy.spin(_node_instance)
    except KeyboardInterrupt:
        pass
    finally:
        cleanup_node()

if __name__ == '__main__':
    main()