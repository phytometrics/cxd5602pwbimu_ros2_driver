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
        
        # Parameters
        self.declare_parameter('velocity_decay', 0.95)  # 速度減衰
        self.declare_parameter('accel_threshold', 0.5)  # 加速度閾値
        self.declare_parameter('max_path_length', 2000)
        self.declare_parameter('update_rate', 50)  # Hz
        self.declare_parameter('stationary_threshold', 0.1)  # 静止判定閾値
        self.declare_parameter('stationary_time', 1.0)  # 静止継続時間
        
        self.velocity_decay = self.get_parameter('velocity_decay').value
        self.accel_threshold = self.get_parameter('accel_threshold').value
        self.max_path_length = self.get_parameter('max_path_length').value
        self.update_rate = self.get_parameter('update_rate').value
        self.stationary_threshold = self.get_parameter('stationary_threshold').value
        self.stationary_time = self.get_parameter('stationary_time').value
        
        # Zero velocity update (ZUPT) variables
        self.stationary_start_time = None
        self.low_accel_count = 0
        self.required_low_accel_count = 20  # 20回連続で低加速度なら静止と判定
        
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
        
        # Apply simple filtering to reduce noise
        filtered_accel = 0.1 * world_accel + 0.9 * self.prev_linear_accel
        
        # Debug: Log acceleration values
        accel_magnitude = np.linalg.norm(filtered_accel)
        if accel_magnitude > 0.1:  # Only log significant accelerations
            self.get_logger().info(f"Filtered accel: [{filtered_accel[0]:.3f}, {filtered_accel[1]:.3f}, {filtered_accel[2]:.3f}], mag: {accel_magnitude:.3f}")
        
        # Zero Velocity Update (ZUPT) - 静止判定
        if accel_magnitude < self.stationary_threshold:
            self.low_accel_count += 1
            if self.low_accel_count >= self.required_low_accel_count:
                # 静止状態と判定 - 速度をゼロにリセット
                self.velocity_x = 0.0
                self.velocity_y = 0.0
                self.velocity_z = 0.0
                if self.low_accel_count == self.required_low_accel_count:
                    self.get_logger().info("Stationary detected - velocity reset to zero")
        else:
            self.low_accel_count = 0
            
            # Only integrate if acceleration is significant
            if accel_magnitude > self.accel_threshold:
                # Integrate acceleration to get velocity
                self.velocity_x += filtered_accel[0] * dt
                self.velocity_y += filtered_accel[1] * dt
                self.velocity_z += filtered_accel[2] * dt
                
                # Debug: Log velocity changes
                vel_magnitude = np.sqrt(self.velocity_x**2 + self.velocity_y**2 + self.velocity_z**2)
                if vel_magnitude > 0.1:
                    self.get_logger().info(f"Velocity: [{self.velocity_x:.3f}, {self.velocity_y:.3f}, {self.velocity_z:.3f}], mag: {vel_magnitude:.3f}")
        
        # Apply velocity decay to prevent drift (only if not stationary)
        if self.low_accel_count < self.required_low_accel_count:
            self.velocity_x *= self.velocity_decay
            self.velocity_y *= self.velocity_decay
            self.velocity_z *= self.velocity_decay
        
        # Integrate velocity to get position
        old_pos_x, old_pos_y, old_pos_z = self.position_x, self.position_y, self.position_z
        self.position_x += self.velocity_x * dt
        self.position_y += self.velocity_y * dt
        self.position_z += self.velocity_z * dt
        
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