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
import tf_transformations
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
        
        self.velocity_decay = self.get_parameter('velocity_decay').value
        self.accel_threshold = self.get_parameter('accel_threshold').value
        self.max_path_length = self.get_parameter('max_path_length').value
        self.update_rate = self.get_parameter('update_rate').value
        
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
        euler = tf_transformations.euler_from_quaternion(quaternion)
        roll, pitch, yaw = euler
        
        # Get linear acceleration and remove gravity using orientation
        linear_accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        # Transform acceleration to world frame (remove gravity)
        # Create rotation matrix from quaternion
        rotation_matrix = tf_transformations.quaternion_matrix(quaternion)[:3, :3]
        
        # Gravity vector in world frame
        gravity_world = np.array([0.0, 0.0, -9.81])
        
        # Transform gravity to body frame
        gravity_body = rotation_matrix.T @ gravity_world
        
        # Remove gravity from acceleration
        world_accel = linear_accel - gravity_body
        
        # Apply simple filtering to reduce noise
        filtered_accel = 0.1 * world_accel + 0.9 * self.prev_linear_accel
        
        # Only integrate if acceleration is significant
        if np.linalg.norm(filtered_accel) > self.accel_threshold:
            # Integrate acceleration to get velocity
            self.velocity_x += filtered_accel[0] * dt
            self.velocity_y += filtered_accel[1] * dt
            self.velocity_z += filtered_accel[2] * dt
        
        # Apply velocity decay to prevent drift
        self.velocity_x *= self.velocity_decay
        self.velocity_y *= self.velocity_decay
        self.velocity_z *= self.velocity_decay
        
        # Integrate velocity to get position
        self.position_x += self.velocity_x * dt
        self.position_y += self.velocity_y * dt
        self.position_z += self.velocity_z * dt
        
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
        if len(self.path.poses) == 0 or self.calculate_distance(pose_stamped, self.path.poses[-1]) > 0.01:
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