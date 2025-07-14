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

class OrientationTrajectoryNode(Node):
    def __init__(self):
        super().__init__('orientation_trajectory_node')
        
        # Subscribe to filtered IMU data from Madgwick filter
        self.imu_subscriber = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        
        # Publishers
        self.trajectory_publisher = self.create_publisher(Path, '/imu/orientation_trajectory', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/imu/orientation_pose', 10)
        
        # Initialize path
        self.path = Path()
        self.path.header.frame_id = "odom"
        
        # Initialize position
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        
        # Previous orientation values
        self.prev_roll = 0.0
        self.prev_pitch = 0.0
        self.prev_yaw = 0.0
        self.first_callback = True
        
        # Parameters
        self.declare_parameter('movement_scale', 1.0)
        self.declare_parameter('max_path_length', 1000)
        
        self.movement_scale = self.get_parameter('movement_scale').value
        self.max_path_length = self.get_parameter('max_path_length').value
        
        self.get_logger().info('Orientation Trajectory Node started')
    
    def imu_callback(self, msg):
        # Get orientation from Madgwick filter
        orientation = msg.orientation
        
        # Convert quaternion to Euler angles
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler = tf_transformations.euler_from_quaternion(quaternion)
        roll, pitch, yaw = euler
        
        # Skip first callback to establish baseline
        if self.first_callback:
            self.prev_roll = roll
            self.prev_pitch = pitch
            self.prev_yaw = yaw
            self.first_callback = False
            return
        
        # Calculate orientation deltas
        d_roll = roll - self.prev_roll
        d_pitch = pitch - self.prev_pitch
        d_yaw = yaw - self.prev_yaw
        
        # Handle angle wrapping
        d_roll = self.normalize_angle(d_roll)
        d_pitch = self.normalize_angle(d_pitch)
        d_yaw = self.normalize_angle(d_yaw)
        
        # Update position based on orientation changes
        self.position_x += d_pitch * self.movement_scale  # Pitch -> X movement
        self.position_y += d_roll * self.movement_scale   # Roll -> Y movement
        self.position_z += d_yaw * self.movement_scale    # Yaw -> Z movement
        
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
        
        # Add to path
        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(pose_stamped)
        
        # Limit path length
        if len(self.path.poses) > self.max_path_length:
            self.path.poses.pop(0)
        
        # Publish trajectory
        self.trajectory_publisher.publish(self.path)
        
        # Update previous values
        self.prev_roll = roll
        self.prev_pitch = pitch
        self.prev_yaw = yaw
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = OrientationTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()