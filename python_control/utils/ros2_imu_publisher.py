#!/usr/bin/env python3
"""
ROS2 IMU Publisher for Spresense CXD5602 Multi-IMU (Fixed Timestamp Version)
Publishes IMU data, quaternions, and trajectory information to ROS2 topics
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
from loguru import logger
import os
import time
import threading
from collections import deque

# ROS2 message types
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped, QuaternionStamped, TransformStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header, Bool
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from builtin_interfaces.msg import Time 

class IMUTrajectoryPublisher(Node):
    """ROS2 publisher for IMU data and trajectory information with fixed timestamps"""
    
    def __init__(self, node_name='imu_trajectory_publisher'):
        super().__init__(node_name)
        
        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Enhanced timestamp synchronization
        self.sync_lock = threading.Lock()
        self.time_offset_ns = None
        self.offset_buffer = deque(maxlen=100)  # Larger buffer for better stability
        self.sync_samples = 0
        self.max_sync_samples = 50  # More samples for better accuracy
        self.last_imu_timestamp = None
        self.time_sync_complete = False
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data', sensor_qos)
        self.pose_pub = self.create_publisher(PoseStamped, '/imu/pose', sensor_qos)
        self.velocity_pub = self.create_publisher(TwistStamped, '/imu/velocity', sensor_qos)
        self.acceleration_pub = self.create_publisher(AccelStamped, '/imu/acceleration', sensor_qos)
        self.path_pub = self.create_publisher(Path, '/imu/trajectory', sensor_qos)
        self.zupt_pub = self.create_publisher(Bool, '/imu/zupt_active', sensor_qos)
        
        # TF broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Path for trajectory visualization
        self.trajectory_path = Path()
        self.trajectory_path.header.frame_id = "odom"
        
        # Parameters
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.imu_frame = self.declare_parameter('imu_frame', 'imu_link').value
        self.max_path_length = self.declare_parameter('max_path_length', 1000).value
        
        # Filtering
        self.last_position = np.array([0.0, 0.0, 0.0])
        self.position_jump_threshold = 2.0
        self.velocity_filter_alpha = 0.9  # Low-pass filter for velocity
        self.filtered_velocity = np.array([0.0, 0.0, 0.0])
        
        # Current pose for TF publishing
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        
        logger.info(f"ROS2 IMU Publisher initialized with frames: {self.imu_frame} -> {self.frame_id} -> {self.odom_frame}")
        
        # Publish static transforms
        self.publish_static_transforms()
        
        # TF publishing timer
        self.tf_timer = self.create_timer(0.02, self.publish_tf)  # 50Hz

    def get_current_timestamp(self):
        """現在のROS2時刻を取得（常にこれを使用）"""
        return self.get_clock().now().to_msg()
    def publish_static_transforms(self):
        """Publish static transform relationships"""
        # Static transform: base_link -> imu_link
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = self.frame_id
        static_tf.child_frame_id = self.imu_frame
        
        # IMU is assumed to be at the center of base_link
        static_tf.transform.translation.x = 0.0
        static_tf.transform.translation.y = 0.0
        static_tf.transform.translation.z = 0.0
        static_tf.transform.rotation.w = 1.0
        static_tf.transform.rotation.x = 0.0
        static_tf.transform.rotation.y = 0.0
        static_tf.transform.rotation.z = 0.0
        
        self.static_tf_broadcaster.sendTransform(static_tf)
        logger.info(f"Published static transform: {self.frame_id} -> {self.imu_frame}")

    def sync_imu_timestamp(self, imu_timestamp_microsec):
        """
        Enhanced IMU timestamp synchronization with ROS2 time
        
        Args:
            imu_timestamp_microsec: IMU timestamp in microseconds since startup
            
        Returns:
            builtin_interfaces.msg.Time: Synchronized ROS time
        """
        with self.sync_lock:
            current_ros_time = self.get_clock().now()
            current_ros_ns = current_ros_time.nanoseconds
            imu_timestamp_ns = imu_timestamp_microsec * 1000  # Convert to nanoseconds
            
            if self.time_offset_ns is None:
                # Initial synchronization
                self.time_offset_ns = current_ros_ns - imu_timestamp_ns
                self.last_imu_timestamp = imu_timestamp_microsec
                logger.info(f"Initial time sync: offset = {self.time_offset_ns}ns")
            
            elif not self.time_sync_complete:
                # Refine synchronization during startup
                if self.last_imu_timestamp is not None:
                    # Calculate drift-compensated offset
                    imu_delta_ns = (imu_timestamp_microsec - self.last_imu_timestamp) * 1000
                    expected_ros_time = self.last_imu_timestamp * 1000 + self.time_offset_ns + imu_delta_ns
                    actual_ros_time = current_ros_ns
                    
                    # Update offset based on drift
                    new_offset = actual_ros_time - imu_timestamp_ns
                    self.offset_buffer.append(new_offset)
                    
                    if len(self.offset_buffer) >= 10:
                        # Use median to reduce outlier impact
                        sorted_offsets = sorted(list(self.offset_buffer))
                        median_offset = sorted_offsets[len(sorted_offsets)//2]
                        self.time_offset_ns = median_offset
                    
                    self.sync_samples += 1
                    
                    if self.sync_samples >= self.max_sync_samples:
                        self.time_sync_complete = True
                        offset_std = np.std(list(self.offset_buffer))
                        logger.info(f"Time sync complete: offset = {self.time_offset_ns}ns, std = {offset_std:.0f}ns")
            
            self.last_imu_timestamp = imu_timestamp_microsec
            
            # Calculate synchronized timestamp
            synced_ns = imu_timestamp_ns + self.time_offset_ns
            
            # Ensure timestamp is not in the future (with small tolerance)
            max_future_tolerance_ns = 100_000_000  # 100ms
            if synced_ns > current_ros_ns + max_future_tolerance_ns:
                logger.warning(f"Future timestamp detected, clamping to current time")
                synced_ns = current_ros_ns
            
            # Convert to ROS Time message
            time_msg = Time()
            time_msg.sec = int(synced_ns // 1_000_000_000)
            time_msg.nanosec = int(synced_ns % 1_000_000_000)
            
            return time_msg

    def publish_tf(self):
        """Publish dynamic TF: odom -> base_link"""
        if not hasattr(self, 'current_position'):
            return
            
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.frame_id
        
        # Use current tracked position
        t.transform.translation.x = float(self.current_position[0])
        t.transform.translation.y = float(self.current_position[1])
        t.transform.translation.z = float(self.current_position[2])
        
        # Use current quaternion
        q_norm = np.linalg.norm(self.current_quaternion)
        if q_norm > 0:
            q = self.current_quaternion / q_norm
        else:
            q = np.array([1.0, 0.0, 0.0, 0.0])
            
        t.transform.rotation.w = float(q[0])
        t.transform.rotation.x = float(q[1])
        t.transform.rotation.y = float(q[2])
        t.transform.rotation.z = float(q[3])
        
        self.tf_broadcaster.sendTransform(t)
        
    def publish_imu_data(self, imu_timestamp_microsec, raw_data, quaternion):
        """Publish raw IMU data with synchronized timestamp"""
        imu_msg = Imu()
        
        # Synchronized timestamp
        # imu_msg.header.stamp = self.sync_imu_timestamp(imu_timestamp_microsec)
        # タイムスタンプ：常に現在時刻（timestamp_microsecは無視）
        imu_msg.header.stamp = self.get_current_timestamp()
        imu_msg.header.frame_id = self.imu_frame
        
        # Normalize quaternion
        q_norm = np.linalg.norm(quaternion)
        if q_norm > 0:
            q_normalized = quaternion / q_norm
        else:
            q_normalized = np.array([1.0, 0.0, 0.0, 0.0])
            
        imu_msg.orientation.w = float(q_normalized[0])
        imu_msg.orientation.x = float(q_normalized[1])
        imu_msg.orientation.y = float(q_normalized[2])
        imu_msg.orientation.z = float(q_normalized[3])
        
        # Store current quaternion for TF
        self.current_quaternion = q_normalized.copy()
        
        # Angular velocity (already in rad/s from your code)
        gyro = raw_data['gyro']
        imu_msg.angular_velocity.x = float(gyro[0])
        imu_msg.angular_velocity.y = float(gyro[1])
        imu_msg.angular_velocity.z = float(gyro[2])
        
        # Linear acceleration
        acc = raw_data['acc']
        imu_msg.linear_acceleration.x = float(acc[0])
        imu_msg.linear_acceleration.y = float(acc[1])
        imu_msg.linear_acceleration.z = float(acc[2])
        
        # Enhanced covariance matrices
        # Orientation covariance (based on IMU quality)
        orientation_variance = 0.01  # rad^2
        imu_msg.orientation_covariance = [0.0] * 9
        imu_msg.orientation_covariance[0] = orientation_variance  # x
        imu_msg.orientation_covariance[4] = orientation_variance  # y
        imu_msg.orientation_covariance[8] = orientation_variance  # z
        
        # Angular velocity covariance
        gyro_variance = 0.001  # (rad/s)^2
        imu_msg.angular_velocity_covariance = [0.0] * 9
        imu_msg.angular_velocity_covariance[0] = gyro_variance
        imu_msg.angular_velocity_covariance[4] = gyro_variance
        imu_msg.angular_velocity_covariance[8] = gyro_variance
        
        # Linear acceleration covariance
        acc_variance = 0.01  # (m/s^2)^2
        imu_msg.linear_acceleration_covariance = [0.0] * 9
        imu_msg.linear_acceleration_covariance[0] = acc_variance
        imu_msg.linear_acceleration_covariance[4] = acc_variance
        imu_msg.linear_acceleration_covariance[8] = acc_variance
        
        self.imu_pub.publish(imu_msg)
        
    def is_position_valid(self, position):
        """Enhanced position validation"""
        if np.any(np.isnan(position)) or np.any(np.isinf(position)):
            return False
            
        # Check for unreasonable positions (more than 100m from origin)
        if np.linalg.norm(position) > 100.0:
            return False
            
        # Check for sudden jumps
        distance = np.linalg.norm(position - self.last_position)
        if distance > self.position_jump_threshold:
            logger.warning(f"Position jump detected: {distance:.3f}m")
            return False
            
        return True
        
    def publish_trajectory_data(self, imu_timestamp_microsec, raw_data):
        """Publish trajectory-related data with enhanced filtering"""
        if 'trajectory' not in raw_data:
            return
            
        trajectory = raw_data['trajectory']
        timestamp = self.sync_imu_timestamp(imu_timestamp_microsec)
        
        # Extract and validate data
        position = np.array(trajectory['position'])
        velocity = np.array(trajectory['velocity'])
        acceleration = np.array(trajectory['acceleration'])
        
        # Position validation and filtering
        if not self.is_position_valid(position):
            logger.warning(f"Invalid position: {position}, skipping trajectory update")
            return
            
        # Update current position for TF
        self.current_position = position.copy()
        self.last_position = position.copy()
        
        # Velocity filtering
        if np.any(np.isnan(velocity)) or np.any(np.isinf(velocity)):
            velocity = np.array([0.0, 0.0, 0.0])
        else:
            # Apply low-pass filter to velocity
            self.filtered_velocity = (self.velocity_filter_alpha * self.filtered_velocity + 
                                    (1.0 - self.velocity_filter_alpha) * velocity)
            velocity = self.filtered_velocity.copy()
        
        # Acceleration validation
        if np.any(np.isnan(acceleration)) or np.any(np.isinf(acceleration)):
            acceleration = np.array([0.0, 0.0, 0.0])
        
        # Clamp extreme values
        velocity_norm = np.linalg.norm(velocity)
        if velocity_norm > 10.0:
            velocity = velocity * 10.0 / velocity_norm
            
        accel_norm = np.linalg.norm(acceleration)
        if accel_norm > 50.0:
            acceleration = acceleration * 50.0 / accel_norm
        
        # Publish pose in odom frame
        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = self.odom_frame
        
        pose_msg.pose.position.x = float(position[0])
        pose_msg.pose.position.y = float(position[1])
        pose_msg.pose.position.z = float(position[2])
        
        # Use current quaternion
        q = self.current_quaternion
        pose_msg.pose.orientation.w = float(q[0])
        pose_msg.pose.orientation.x = float(q[1])
        pose_msg.pose.orientation.y = float(q[2])
        pose_msg.pose.orientation.z = float(q[3])
        
        self.pose_pub.publish(pose_msg)
        
        # Publish velocity in base_link frame
        velocity_msg = TwistStamped()
        velocity_msg.header.stamp = timestamp
        velocity_msg.header.frame_id = self.frame_id
        
        velocity_msg.twist.linear.x = float(velocity[0])
        velocity_msg.twist.linear.y = float(velocity[1])
        velocity_msg.twist.linear.z = float(velocity[2])
        
        self.velocity_pub.publish(velocity_msg)
        
        # Publish acceleration in base_link frame
        accel_msg = AccelStamped()
        accel_msg.header.stamp = timestamp
        accel_msg.header.frame_id = self.frame_id
        
        accel_msg.accel.linear.x = float(acceleration[0])
        accel_msg.accel.linear.y = float(acceleration[1])
        accel_msg.accel.linear.z = float(acceleration[2])
        
        self.acceleration_pub.publish(accel_msg)
        
        # Update trajectory path
        self.trajectory_path.header.stamp = timestamp
        self.trajectory_path.poses.append(pose_msg)
        
        # Limit path length
        if len(self.trajectory_path.poses) > self.max_path_length:
            self.trajectory_path.poses.pop(0)
            
        self.path_pub.publish(self.trajectory_path)
        
        # Publish ZUPT status
        if 'zupt_status' in trajectory:
            zupt_msg = Bool()
            zupt_msg.data = trajectory['zupt_status'].get('is_static', False)
            self.zupt_pub.publish(zupt_msg)
        
    def clear_trajectory(self):
        """Clear the trajectory path"""
        self.trajectory_path.poses.clear()
        self.last_position = np.array([0.0, 0.0, 0.0])
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.filtered_velocity = np.array([0.0, 0.0, 0.0])
        logger.info("Trajectory path cleared")


class IMUTrajectoryROS2Bridge:
    """Enhanced bridge between IMU processor and ROS2 publisher"""
    
    def __init__(self, node_name='imu_trajectory_bridge'):
        # Initialize ROS2
        if not rclpy.ok():
            rclpy.init()
            
        self.node = IMUTrajectoryPublisher(node_name)
        self.is_spinning = False
        self.spin_thread = None
        
    def start_spinning(self):
        """Start ROS2 spinning in a separate thread"""
        if not self.is_spinning:
            self.is_spinning = True
            self.spin_thread = threading.Thread(target=self._spin_thread, daemon=True)
            self.spin_thread.start()
            logger.info("ROS2 spinning started in background thread")
    
    def _spin_thread(self):
        """ROS2 spin thread function with error handling"""
        try:
            while self.is_spinning and rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception as e:
            logger.error(f"ROS2 spin error: {e}")
        finally:
            self.is_spinning = False
    
    def stop_spinning(self):
        """Stop ROS2 spinning"""
        if self.is_spinning:
            self.is_spinning = False
            if self.spin_thread and self.spin_thread.is_alive():
                self.spin_thread.join(timeout=2.0)
            
            try:
                self.node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception as e:
                logger.warning(f"Error during ROS2 shutdown: {e}")
            
            logger.info("ROS2 spinning stopped")
    
    def quaternion_callback(self, quaternion, timestamp_microsec, raw_data):
        """Enhanced callback function for IMU processor"""
        try:
            if not self.is_spinning:
                return
                
            # Store quaternion in raw_data for pose publishing
            raw_data['quaternion'] = quaternion
            
            # Publish IMU data
            self.node.publish_imu_data(timestamp_microsec, raw_data, quaternion)
            
            # Publish trajectory data if available
            self.node.publish_trajectory_data(timestamp_microsec, raw_data)
            
        except Exception as e:
            logger.error(f"Error in ROS2 callback: {e}")
    
    def clear_trajectory(self):
        """Clear trajectory path"""
        if hasattr(self.node, 'clear_trajectory'):
            self.node.clear_trajectory()
    
    def __enter__(self):
        self.start_spinning()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop_spinning()


def main():
    """Main function for standalone ROS2 node"""
    rclpy.init()
    
    node = IMUTrajectoryPublisher()
    
    try:
        logger.info("ROS2 IMU Trajectory Publisher node started")
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Shutting down ROS2 IMU Trajectory Publisher")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()