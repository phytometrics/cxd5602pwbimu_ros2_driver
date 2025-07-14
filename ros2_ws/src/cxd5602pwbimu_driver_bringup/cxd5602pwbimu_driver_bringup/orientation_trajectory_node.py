#!/usr/bin/env python3
"""
Copyright (c) 2025 NITK.K ROS-Team

SPDX-License-Identifier: Apache-2.0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
try:
    from tf_transformations import euler_from_quaternion, quaternion_matrix
except ImportError:
    # Fallback for ROS2 Humble
    from scipy.spatial.transform import Rotation as R
    def euler_from_quaternion(quaternion):
        """Convert quaternion to Euler angles"""
        rotation = R.from_quat([quaternion[0], quaternion[1], quaternion[2], quaternion[3]])
        return rotation.as_euler('xyz', degrees=False)
    
    def quaternion_matrix(quaternion):
        """Convert quaternion to rotation matrix"""
        rotation = R.from_quat([quaternion[0], quaternion[1], quaternion[2], quaternion[3]])
        matrix = np.eye(4)
        matrix[:3, :3] = rotation.as_matrix()
        return matrix
import math
import numpy as np
import signal
import sys
import atexit

class RobotTrajectoryNode(Node):
    def __init__(self):
        super().__init__('robot_trajectory_node')
        
        # Subscribe to filtered IMU data from Madgwick filter
        self.imu_subscriber = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        
        # Publishers
        self.trajectory_publisher = self.create_publisher(Path, '/robot/trajectory', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/robot/pose', 10)
        
        # Initialize path
        self.path = Path()
        self.path.header.frame_id = "odom"
        
        # Initialize position
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        
        # Initialize velocity
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_z = 0.0
        
        # Previous values
        self.prev_time = None
        self.prev_linear_accel = np.array([0.0, 0.0, 0.0])
        
        # Parameters optimized for high-precision Sony Multi-IMU
        self.declare_parameter('velocity_decay', 0.9995)  # 高精度IMU用の緩い減衰
        self.declare_parameter('accel_threshold', 0.02)  # 高精度なので低閾値
        self.declare_parameter('max_path_length', 5000)
        self.declare_parameter('update_rate', 100)  # Hz
        self.declare_parameter('stationary_threshold', 0.05)  # 高精度静止判定
        self.declare_parameter('stationary_time', 0.5)  # 短い静止判定時間
        self.declare_parameter('bias_estimation_window', 100)  # バイアス推定窓
        
        self.velocity_decay = self.get_parameter('velocity_decay').value
        self.accel_threshold = self.get_parameter('accel_threshold').value
        self.max_path_length = self.get_parameter('max_path_length').value
        self.update_rate = self.get_parameter('update_rate').value
        self.stationary_threshold = self.get_parameter('stationary_threshold').value
        self.stationary_time = self.get_parameter('stationary_time').value
        self.bias_estimation_window = self.get_parameter('bias_estimation_window').value
        
        # Advanced filtering for high-precision IMU
        self.accel_history = []
        self.bias_estimate = np.array([0.0, 0.0, 0.0])
        self.low_accel_count = 0
        self.required_low_accel_count = 10  # 高精度なので短時間判定
        
        # Enhanced trajectory correction
        self.position_history = []
        self.velocity_history = []
        self.last_stationary_position = np.array([0.0, 0.0, 0.0])
        self.last_stationary_time = None
        self.trajectory_correction_factor = 0.98  # 軌跡補正係数
        
        # Movement detection
        self.movement_started = False
        self.movement_threshold = 0.1  # 移動検出閾値
        self.return_detection_window = 50  # 復帰検出窓
        
        self.get_logger().info('Robot Trajectory Node started')
    
    def imu_callback(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Skip first callback to establish baseline
        if self.prev_time is None:
            self.prev_time = current_time
            return
            
        # Calculate time delta
        dt = current_time - self.prev_time
        if dt <= 0 or dt > 0.1:  # Skip invalid time deltas
            self.prev_time = current_time
            return
        
        # Get orientation from Madgwick filter
        orientation = msg.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler = euler_from_quaternion(quaternion)
        roll, pitch, yaw = euler
        
        # Get linear acceleration and remove gravity using orientation
        linear_accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        # Transform acceleration to world frame (remove gravity)
        # Create rotation matrix from quaternion
        rotation_matrix = quaternion_matrix(quaternion)[:3, :3]
        
        # Transform acceleration from body frame to world frame
        world_accel = rotation_matrix @ linear_accel
        
        # Remove gravity in world frame (Z-axis is up)
        world_accel[2] -= 9.81  # Remove gravity from Z-axis
        
        # Debug: Log raw and transformed acceleration
        if np.linalg.norm(world_accel) > 0.1:
            self.get_logger().info(f"Raw accel: [{linear_accel[0]:.3f}, {linear_accel[1]:.3f}, {linear_accel[2]:.3f}]")
            self.get_logger().info(f"World accel: [{world_accel[0]:.3f}, {world_accel[1]:.3f}, {world_accel[2]:.3f}]")
        
        # Advanced bias estimation for high-precision IMU
        self.accel_history.append(world_accel.copy())
        if len(self.accel_history) > self.bias_estimation_window:
            self.accel_history.pop(0)
        
        # Estimate bias during stationary periods
        if len(self.accel_history) >= self.bias_estimation_window:
            recent_accels = np.array(self.accel_history[-50:])  # Last 50 samples
            if np.all(np.std(recent_accels, axis=0) < 0.1):  # Low variation = stationary
                self.bias_estimate = np.mean(recent_accels, axis=0)
        
        # Remove bias estimate
        bias_corrected_accel = world_accel - self.bias_estimate
        
        # Apply sophisticated filtering for high-precision IMU
        # Use lower smoothing factor for high-precision sensors
        filtered_accel = 0.3 * bias_corrected_accel + 0.7 * self.prev_linear_accel
        
        # Debug: Log acceleration values including bias
        accel_magnitude = np.linalg.norm(filtered_accel)
        if accel_magnitude > 0.05:  # Lower threshold for high-precision IMU
            self.get_logger().info(f"Bias estimate: [{self.bias_estimate[0]:.4f}, {self.bias_estimate[1]:.4f}, {self.bias_estimate[2]:.4f}]")
            self.get_logger().info(f"Filtered accel: [{filtered_accel[0]:.4f}, {filtered_accel[1]:.4f}, {filtered_accel[2]:.4f}], mag: {accel_magnitude:.4f}")
        
        # Enhanced Zero Velocity Update (ZUPT) for high-precision IMU
        if accel_magnitude < self.stationary_threshold:
            self.low_accel_count += 1
            if self.low_accel_count >= self.required_low_accel_count:
                # 静止状態と判定 - 速度をゼロにリセット
                self.velocity_x = 0.0
                self.velocity_y = 0.0
                self.velocity_z = 0.0
                if self.low_accel_count == self.required_low_accel_count:
                    self.get_logger().info("High-precision stationary detected - velocity reset to zero")
        else:
            self.low_accel_count = 0
            
            # High-precision IMU can integrate even small accelerations
            if accel_magnitude > self.accel_threshold:
                # Use more sophisticated integration (Trapezoidal rule)
                # For high-precision IMU, we can trust smaller accelerations
                self.velocity_x += filtered_accel[0] * dt
                self.velocity_y += filtered_accel[1] * dt
                self.velocity_z += filtered_accel[2] * dt
                
                # Debug: Log velocity changes
                vel_magnitude = np.sqrt(self.velocity_x**2 + self.velocity_y**2 + self.velocity_z**2)
                if vel_magnitude > 0.05:
                    self.get_logger().info(f"Velocity: [{self.velocity_x:.4f}, {self.velocity_y:.4f}, {self.velocity_z:.4f}], mag: {vel_magnitude:.4f}")
        
        # Apply minimal velocity decay for high-precision IMU (trust the sensor more)
        if self.low_accel_count < self.required_low_accel_count:
            self.velocity_x *= self.velocity_decay
            self.velocity_y *= self.velocity_decay
            self.velocity_z *= self.velocity_decay
        
        # Integrate velocity to get position
        old_pos_x, old_pos_y, old_pos_z = self.position_x, self.position_y, self.position_z
        self.position_x += self.velocity_x * dt
        self.position_y += self.velocity_y * dt
        self.position_z += self.velocity_z * dt
        
        # Advanced trajectory correction
        current_position = np.array([self.position_x, self.position_y, self.position_z])
        
        # Record position history
        self.position_history.append(current_position.copy())
        if len(self.position_history) > 200:  # Keep last 200 positions
            self.position_history.pop(0)
        
        # Check if we've returned to near the starting position
        if len(self.position_history) > self.return_detection_window:
            # If we're stationary and close to start, apply correction
            if self.low_accel_count >= self.required_low_accel_count:
                distance_from_start = np.linalg.norm(current_position - self.last_stationary_position)
                
                # If we're close to the last stationary position, apply correction
                if distance_from_start < 0.5 and self.movement_started:
                    correction_factor = 1.0 - (distance_from_start / 0.5) * 0.3
                    self.position_x *= correction_factor
                    self.position_y *= correction_factor
                    self.position_z *= correction_factor
                    
                    self.get_logger().info(f"Return trajectory correction applied: factor={correction_factor:.3f}, distance={distance_from_start:.3f}")
                    
                    # Update last stationary position
                    self.last_stationary_position = np.array([self.position_x, self.position_y, self.position_z])
                    self.movement_started = False
                
                # Record stationary position
                if self.low_accel_count == self.required_low_accel_count:
                    self.last_stationary_position = current_position.copy()
                    
        # Detect movement start
        if not self.movement_started and accel_magnitude > self.movement_threshold:
            self.movement_started = True
            self.get_logger().info("Movement detected - starting trajectory tracking")
        
        # Debug: Log position changes
        pos_change = np.sqrt((self.position_x - old_pos_x)**2 + (self.position_y - old_pos_y)**2 + (self.position_z - old_pos_z)**2)
        if pos_change > 0.01:
            self.get_logger().info(f"Position: [{self.position_x:.3f}, {self.position_y:.3f}, {self.position_z:.3f}], change: {pos_change:.3f}")
        
        # Create pose stamped message
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = msg.header.stamp
        pose_stamped.header.frame_id = "odom"
        pose_stamped.pose.position.x = self.position_x
        pose_stamped.pose.position.y = self.position_y
        pose_stamped.pose.position.z = self.position_z
        pose_stamped.pose.orientation = msg.orientation
        
        # Publish current pose
        self.pose_publisher.publish(pose_stamped)
        
        # Add to path (only if significant movement)
        if len(self.path.poses) == 0 or self.calculate_distance(pose_stamped, self.path.poses[-1]) > 0.001:
            self.path.header.stamp = msg.header.stamp
            self.path.poses.append(pose_stamped)
            
            # Limit path length
            if len(self.path.poses) > self.max_path_length:
                self.path.poses.pop(0)
        
        # Publish trajectory
        self.trajectory_publisher.publish(self.path)
        
        # Update previous values
        self.prev_time = current_time
        self.prev_linear_accel = filtered_accel
    
    def calculate_distance(self, pose1, pose2):
        """Calculate Euclidean distance between two poses"""
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        dz = pose1.pose.position.z - pose2.pose.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

# Global variable for node cleanup
_node_instance = None

def cleanup_node():
    """Clean up ROS2 node resources"""
    global _node_instance
    if _node_instance is not None:
        print("\nShutting down ROS2 node...")
        _node_instance.destroy_node()
        _node_instance = None
    
    if rclpy.ok():
        rclpy.shutdown()

def signal_handler(signum, frame):
    """Handle SIGINT and SIGTERM signals"""
    print(f"\nReceived signal {signum}, shutting down...")
    cleanup_node()
    sys.exit(0)

def main(args=None):
    global _node_instance
    
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Register cleanup function for normal exit
    atexit.register(cleanup_node)
    
    try:
        rclpy.init(args=args)
        _node_instance = RobotTrajectoryNode()
        
        print("Robot trajectory node started. Press Ctrl+C to stop.")
        rclpy.spin(_node_instance)
        
    except KeyboardInterrupt:
        print("\nStopped by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        cleanup_node()

if __name__ == '__main__':
    main()