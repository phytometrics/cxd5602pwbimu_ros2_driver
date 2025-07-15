#!/usr/bin/env python3
"""
Advanced Python INS Processor for Spresense CXD5602 Multi-IMU
Real-time Inertial Navigation System with Quaternion-based attitude estimation

Enhanced Frame Structure (44 bytes total):
  0     : 'Z'                         Header (enhanced minimal format)
  1-4   : uint32  sec                 System time (seconds)
  5-8   : uint32  msec                System time (milliseconds)
  9-20  : float32 ax,ay,az            Calibrated acceleration [m/s²]
 21-32  : float32 gx,gy,gz            Calibrated angular velocity [deg/s]
 33-36  : float32 temp                Temperature [°C]
 37-40  : uint32  hw_timestamp        Hardware timestamp
 41     : uint8   CRC8                Dallas/Maxim CRC
 42-43  : '\r''\n'                   Frame delimiter

Features:
- Real-time quaternion-based attitude estimation
- Advanced sensor fusion with Madgwick/Mahony filters
- Zero velocity update (ZUPT) for drift correction
- Position and velocity tracking
- Bias estimation and compensation
"""

import struct
import serial
import crc8
import signal
import sys
import atexit
import numpy as np
import time
import math
from dataclasses import dataclass
from typing import Optional, Tuple, List
from collections import deque
import threading
import queue

# ========= Environment Settings =========================================
PORT     = "/dev/cu.usbserial-1130"   # ← Change to your Spresense port
BAUDRATE = 115_200
TIMEOUT  = 1.0                        # [s] read() timeout
# ===========================================================

@dataclass
class EnhancedIMUData:
    """Enhanced IMU data structure"""
    timestamp: float
    hw_timestamp: int
    acceleration: np.ndarray     # [ax, ay, az] in m/s²
    angular_velocity: np.ndarray # [gx, gy, gz] in rad/s (converted from deg/s)
    temperature: float           # Temperature in °C

@dataclass
class INSState:
    """Complete INS state"""
    timestamp: float
    quaternion: np.ndarray       # [qw, qx, qy, qz] - attitude quaternion
    velocity: np.ndarray         # [vx, vy, vz] in m/s (global frame)
    position: np.ndarray         # [px, py, pz] in m (global frame)
    accel_bias: np.ndarray       # [bx, by, bz] accelerometer bias
    gyro_bias: np.ndarray        # [bx, by, bz] gyroscope bias
    gravity_vector: np.ndarray   # [gx, gy, gz] gravity in body frame

def crc8_maxim(data: bytes) -> int:
    """Dallas/Maxim CRC-8"""
    h = crc8.crc8()
    h.update(data)
    return h.digest()[0]

# Frame structures
BASIC_STRUCT = struct.Struct("<cII6fB")        # X sec msec ax ay az gx gy gz crc (34 bytes)
ENHANCED_STRUCT = struct.Struct("<cII9fIB")    # Z sec msec ax ay az gx gy gz temp hw_timestamp crc (42 bytes)
BASIC_FRAME_SIZE = 36                          # 34 + CRLF(2)
ENHANCED_FRAME_SIZE = 44                       # 42 + CRLF(2)

# Global variables
_serial_port = None
_ins_processor = None
_stats = {
    'total_frames': 0,
    'crc_errors': 0,
    'sync_errors': 0,
    'ins_outputs': 0,
    'start_time': time.time()
}

def parse_frame(payload: bytes) -> Optional[EnhancedIMUData]:
    """Parse frame - supports both basic (X) and enhanced (Z) formats"""
    if len(payload) == 0:
        return None
        
    header_byte = payload[0:1]
    
    if header_byte == b"X" and len(payload) == 34:
        return parse_basic_frame(payload)
    elif header_byte == b"Z" and len(payload) == 42:
        return parse_enhanced_frame(payload)
    else:
        return None

def parse_basic_frame(payload: bytes) -> Optional[EnhancedIMUData]:
    """Parse basic IMU frame (header 'X') - legacy support"""
    try:
        unpacked = BASIC_STRUCT.unpack(payload)
        header = unpacked[0]
        sec, msec = unpacked[1:3]
        ax, ay, az, gx, gy, gz = unpacked[3:9]
        crc_recv = unpacked[9]
        
        if header != b"X":
            return None
            
        # Verify CRC
        crc_calc = crc8_maxim(payload[:-1])
        if crc_calc != crc_recv:
            _stats['crc_errors'] += 1
            return None
            
        timestamp = sec + msec / 1000.0
        acceleration = np.array([ax, ay, az])
        angular_velocity = np.array([np.radians(gx), np.radians(gy), np.radians(gz)])
        
        return EnhancedIMUData(
            timestamp=timestamp,
            hw_timestamp=0,  # Not available in basic format
            acceleration=acceleration,
            angular_velocity=angular_velocity,
            temperature=25.0  # Default temperature
        )
        
    except struct.error:
        return None

def parse_enhanced_frame(payload: bytes) -> Optional[EnhancedIMUData]:
    """Parse enhanced IMU frame (header 'Z')"""
    try:
        if len(payload) != 42:
            return None
            
        unpacked = ENHANCED_STRUCT.unpack(payload)
        header = unpacked[0]
        sec, msec = unpacked[1:3]
        ax, ay, az, gx, gy, gz, temp = unpacked[3:10]
        hw_timestamp = unpacked[10]
        crc_recv = unpacked[11]
        
        if header != b"Z":
            return None
            
        # Verify CRC
        crc_calc = crc8_maxim(payload[:-1])
        if crc_calc != crc_recv:
            _stats['crc_errors'] += 1
            return None
            
        timestamp = sec + msec / 1000.0
        acceleration = np.array([ax, ay, az])
        angular_velocity = np.array([np.radians(gx), np.radians(gy), np.radians(gz)])
        
        return EnhancedIMUData(
            timestamp=timestamp,
            hw_timestamp=hw_timestamp,
            acceleration=acceleration,
            angular_velocity=angular_velocity,
            temperature=temp
        )
        
    except struct.error:
        return None

# INS Parameters (based on reference code)
GRAVITY_MAGNITUDE = 9.80665             # Standard gravity [m/s²]
EARTH_ROTATION_RATE = 7.2921159e-5     # Earth rotation rate [rad/s]
MEASUREMENT_FREQUENCY = 1920            # Expected sensor frequency [Hz]
LATITUDE_JAPAN = 35.0                   # Japan approximate latitude [degrees]

# Reference code constants
GYRO_NOISE_DENSITY = 1.0e-3 * np.pi / 180.0
ACCEL_NOISE_DENSITY = 14.0e-6 * GRAVITY_MAGNITUDE
GYRO_NOISE_AMOUNT = GYRO_NOISE_DENSITY * np.sqrt(MEASUREMENT_FREQUENCY)
ACCEL_NOISE_AMOUNT = ACCEL_NOISE_DENSITY * np.sqrt(MEASUREMENT_FREQUENCY)
ACCEL_BIAS_DRIFT = 4.43e-6 * GRAVITY_MAGNITUDE * 3.0
GYRO_BIAS_DRIFT = 0.39 * np.pi / 180.0

# Filter parameters (from reference code)
MADGWICK_BETA = 0.11                    # ACC_MADGWICK_FILTER_WEIGHT
MADGWICK_GYRO_BETA = 0.00000001        # GYRO_MADGWICK_FILTER_WEIGHT
MAHONY_KP = 0.3                         
MAHONY_KI = 0.0                         
BIAS_ESTIMATION_SAMPLES = 200           

# Static detection parameters (based on reference calibration conditions)
STATIC_ACCEL_THRESHOLD = ACCEL_NOISE_AMOUNT * 40     # Reference uses *40
STATIC_GYRO_THRESHOLD = EARTH_ROTATION_RATE * 2.0    # Reference uses *2.0
STATIC_MIN_SAMPLES = 20                               # Reduced for responsiveness
LIST_SIZE = 8                                         # Gaussian filter size from reference
SIGMA_K = LIST_SIZE / 8.0                            # Gaussian filter sigmaSAMPLES = 200           # Reduced from 1000 for faster startup

@dataclass
class EnhancedIMUData:
    """Enhanced IMU data structure"""
    timestamp: float
    hw_timestamp: int
    acceleration: np.ndarray     # [ax, ay, az] in m/s²
    angular_velocity: np.ndarray # [gx, gy, gz] in rad/s (converted from deg/s)
    temperature: float           # Temperature in °C

@dataclass
class INSState:
    """Complete INS state"""
    timestamp: float
    quaternion: np.ndarray       # [qw, qx, qy, qz] - attitude quaternion
    velocity: np.ndarray         # [vx, vy, vz] in m/s (global frame)
    position: np.ndarray         # [px, py, pz] in m (global frame)
    accel_bias: np.ndarray       # [bx, by, bz] accelerometer bias
    gyro_bias: np.ndarray        # [bx, by, bz] gyroscope bias
    gravity_vector: np.ndarray   # [gx, gy, gz] gravity in body frame

class AdvancedINSProcessor:
    """Advanced INS processor based on reference implementation"""
    
    def __init__(self, filter_type='madgwick'):
        self.filter_type = filter_type
        self.initialized = False
        
        # State variables
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.position = np.array([0.0, 0.0, 0.0])
        self.accel_bias = np.array([0.0, 0.0, 0.0])
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        self.current_gravity = np.array([0.0, 0.0, GRAVITY_MAGNITUDE])
        
        # Filter state
        self.mahony_integral_fb = np.array([0.0, 0.0, 0.0])
        self.last_timestamp = None
        
        # Initialization buffers
        self.init_samples = deque(maxlen=BIAS_ESTIMATION_SAMPLES)
        
        # Zero velocity update (from reference code)
        self.biased_velocity = 0.0
        self.zero_velocity_counter = 0
        
        # Circular buffers for Gaussian filtering (from reference)
        self.measured_accel = deque(maxlen=LIST_SIZE)
        self.measured_gyro = deque(maxlen=LIST_SIZE)
        
        # Calibration state
        self.calibrate_counter = 0
        
        # Statistics
        self.sample_count = 0
        
        # Gaussian filter kernel (from reference implementation)
        self.gaussian_kernel = self._init_gaussian_kernel()
        
    def _init_gaussian_kernel(self):
        """Initialize Gaussian filter kernel based on reference code"""
        kernel = np.zeros(LIST_SIZE)
        total = 0.0
        
        for i in range(LIST_SIZE):
            kernel[i] = (1.0 / (np.sqrt(2.0 * np.pi) * SIGMA_K)) * \
                       np.exp(-(i * i) / (2.0 * SIGMA_K * SIGMA_K))
            total += kernel[i]
        
        # Normalize
        kernel = kernel / total
        return kernel
        
    def apply_gaussian_filter(self, data_buffer, axis):
        """Apply causal Gaussian filter (from reference code)"""
        if len(data_buffer) < LIST_SIZE:
            return data_buffer[-1][axis] if data_buffer else 0.0
            
        y_current = 0.0
        for i in range(LIST_SIZE):
            idx = LIST_SIZE - 1 - i
            list_idx = len(data_buffer) - 1 - idx
            if list_idx >= 0:
                y_current += self.gaussian_kernel[i] * data_buffer[list_idx][axis]
        
        return y_current
        
    def quaternion_multiply(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])
        
    def quaternion_conjugate(self, q: np.ndarray) -> np.ndarray:
        """Get quaternion conjugate"""
        return np.array([q[0], -q[1], -q[2], -q[3]])
        
    def quaternion_normalize(self, q: np.ndarray) -> np.ndarray:
        """Normalize quaternion"""
        norm = np.linalg.norm(q)
        return q / norm if norm > 0 else np.array([1.0, 0.0, 0.0, 0.0])
        
    def rotate_vector(self, v: np.ndarray, q: np.ndarray) -> np.ndarray:
        """Rotate vector by quaternion"""
        v_quat = np.array([0.0, v[0], v[1], v[2]])
        q_conj = self.quaternion_conjugate(q)
        result = self.quaternion_multiply(self.quaternion_multiply(q, v_quat), q_conj)
        return result[1:4]
        
    def quaternion_to_euler(self, q: np.ndarray) -> np.ndarray:
        """Convert quaternion to Euler angles (roll, pitch, yaw) in degrees"""
        w, x, y, z = q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return np.degrees([roll, pitch, yaw])
        
    def madgwick_update(self, gyro: np.ndarray, accel: np.ndarray, dt: float):
        """Madgwick AHRS algorithm update"""
        q = self.quaternion.copy()
        
        # Normalize accelerometer measurement
        if np.linalg.norm(accel) == 0:
            return
        accel = accel / np.linalg.norm(accel)
        
        # Gradient descent algorithm corrective step
        f = np.array([
            2*(q[1]*q[3] - q[0]*q[2]) - accel[0],
            2*(q[0]*q[1] + q[2]*q[3]) - accel[1],
            2*(0.5 - q[1]**2 - q[2]**2) - accel[2]
        ])
        
        j = np.array([
            [-2*q[2], 2*q[3], -2*q[0], 2*q[1]],
            [2*q[1], 2*q[0], 2*q[3], 2*q[2]],
            [0, -4*q[1], -4*q[2], 0]
        ])
        
        step = j.T @ f
        step = step / np.linalg.norm(step)
        
        # Compute rate of change of quaternion
        q_dot = 0.5 * self.quaternion_multiply(q, np.array([0, gyro[0], gyro[1], gyro[2]]))
        
        # Apply feedback step
        q_dot = q_dot - MADGWICK_BETA * step
        
        # Integrate to yield quaternion
        q = q + q_dot * dt
        self.quaternion = self.quaternion_normalize(q)
        
    def mahony_update(self, gyro: np.ndarray, accel: np.ndarray, dt: float):
        """Mahony AHRS algorithm update"""
        q = self.quaternion.copy()
        
        # Normalize accelerometer measurement
        if np.linalg.norm(accel) == 0:
            return
        accel = accel / np.linalg.norm(accel)
        
        # Estimated direction of gravity in body frame
        v = np.array([
            2*(q[1]*q[3] - q[0]*q[2]),
            2*(q[0]*q[1] + q[2]*q[3]),
            q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2
        ])
        
        # Error is sum of cross product between estimated and measured direction of gravity
        e = np.cross(accel, v)
        
        # Apply proportional feedback
        if MAHONY_KI > 0:
            self.mahony_integral_fb += e * dt
            gyro_corrected = gyro + MAHONY_KP * e + MAHONY_KI * self.mahony_integral_fb
        else:
            gyro_corrected = gyro + MAHONY_KP * e
            
        # Apply feedback to gyroscope
        q_dot = 0.5 * self.quaternion_multiply(q, np.array([0, gyro_corrected[0], gyro_corrected[1], gyro_corrected[2]]))
        
        # Integrate quaternion
        q = q + q_dot * dt
        self.quaternion = self.quaternion_normalize(q)
        
    def zero_velocity_correction(self, velocity, dt):
        """Zero velocity correction based on reference implementation"""
        self.biased_velocity += ACCEL_BIAS_DRIFT * dt
        
        # Check if all velocity components are below threshold
        if (abs(velocity[0]) < self.biased_velocity and
            abs(velocity[1]) < self.biased_velocity and
            abs(velocity[2]) < self.biased_velocity):
            self.zero_velocity_counter += 1
        else:
            self.zero_velocity_counter = 0
            
        # Reset velocity if stationary for more than 1 second
        if self.zero_velocity_counter > MEASUREMENT_FREQUENCY:
            self.biased_velocity = 0.0
            self.zero_velocity_counter = 0
            return True
        return False
        
    def check_calibration_conditions(self, accel_filtered, gyro_filtered):
        """Check calibration conditions based on reference code"""
        accel_norm = np.linalg.norm(accel_filtered)
        
        # Remove component parallel to acceleration (earth rotation extraction)
        dot_product = np.dot(accel_filtered, gyro_filtered)
        accel_unit = accel_filtered / (accel_norm + 1e-10)
        earth_rotation = gyro_filtered - dot_product * accel_unit
        earth_rotation_norm = np.linalg.norm(earth_rotation)
        
        # Reference calibration conditions
        accel_condition = (accel_norm < GRAVITY_MAGNITUDE + STATIC_ACCEL_THRESHOLD and
                          accel_norm > GRAVITY_MAGNITUDE - STATIC_ACCEL_THRESHOLD)
        
        gyro_condition = (earth_rotation_norm < EARTH_ROTATION_RATE + STATIC_GYRO_THRESHOLD and
                         earth_rotation_norm > EARTH_ROTATION_RATE / 2.0)
        
        return accel_condition and gyro_condition, accel_norm, earth_rotation_norm
        
    def madgwick_update_with_conditions(self, gyro: np.ndarray, accel: np.ndarray, dt: float):
        """Madgwick update with reference code conditions"""
        q = self.quaternion.copy()
        
        # Normalize accelerometer measurement
        if np.linalg.norm(accel) == 0:
            return
        accel_norm = accel / np.linalg.norm(accel)
        
        # Check calibration conditions
        can_calibrate, _, _ = self.check_calibration_conditions(accel, gyro)
        
        if can_calibrate:
            self.calibrate_counter = min(self.calibrate_counter + 1, MEASUREMENT_FREQUENCY // 30)
            
            if self.calibrate_counter >= MEASUREMENT_FREQUENCY // 30:  # About 1/30 second
                # Apply Madgwick correction for accelerometer
                f = np.array([
                    2*(q[1]*q[3] - q[0]*q[2]) - accel_norm[0],
                    2*(q[0]*q[1] + q[2]*q[3]) - accel_norm[1],
                    2*(0.5 - q[1]**2 - q[2]**2) - accel_norm[2]
                ])
                
                j = np.array([
                    [-2*q[2], 2*q[3], -2*q[0], 2*q[1]],
                    [2*q[1], 2*q[0], 2*q[3], 2*q[2]],
                    [0, -4*q[1], -4*q[2], 0]
                ])
                
                step = j.T @ f
                if np.linalg.norm(step) > 0:
                    step = step / np.linalg.norm(step)
                    
                    # Apply correction with reference weights
                    for i in range(4):
                        q[i] -= step[i] * MADGWICK_BETA * dt
                
                # Update gravity vector
                gyro_minus = -gyro
                self.current_gravity = self.update_vector_rk4(self.current_gravity, gyro_minus, dt)
                
                # Reset velocity and position during calibration
                self.velocity *= 0.95  # Gradual reset
                self.calibrate_counter = 0
        else:
            self.calibrate_counter = 0
            
        # Regular gyroscope integration
        q_dot = 0.5 * self.quaternion_multiply(q, np.array([0, gyro[0], gyro[1], gyro[2]]))
        q = q + q_dot * dt
        
        self.quaternion = self.quaternion_normalize(q)
        
    def update_vector_rk4(self, v, omega, h):
        """RK4 vector update from reference code"""
        k1 = np.cross(omega, v)
        k2 = np.cross(omega, v + (h/2)*k1)
        k3 = np.cross(omega, v + (h/2)*k2) 
        k4 = np.cross(omega, v + h*k3)
        
        v_next = v + (h/6)*(k1 + 2*k2 + 2*k3 + k4)
        return v_next
                
    def estimate_bias(self, data: EnhancedIMUData):
        """Estimate sensor biases during initialization - based on reference code"""
        self.init_samples.append((data.acceleration, data.angular_velocity))
        
        if len(self.init_samples) >= BIAS_ESTIMATION_SAMPLES:
            # Calculate bias from collected samples
            accel_samples = np.array([s[0] for s in self.init_samples])
            gyro_samples = np.array([s[1] for s in self.init_samples])
            
            # Gyro bias is simply the mean (assuming stationary)
            self.gyro_bias = np.mean(gyro_samples, axis=0)
            
            # Accel bias: remove gravity component
            accel_mean = np.mean(accel_samples, axis=0)
            accel_magnitude = np.linalg.norm(accel_mean)
            
            # Initialize gravity vector based on measured acceleration
            self.current_gravity = accel_mean.copy()
            
            # Simple initial orientation based on gravity direction
            gravity_unit = accel_mean / accel_magnitude
            
            # Calculate quaternion to rotate [0,0,-1] to gravity_unit
            z_down = np.array([0, 0, -1])  # Reference assumes Z points down
            
            if abs(np.dot(gravity_unit, z_down)) > 0.99:
                # Nearly aligned, use identity
                self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])
            else:
                # Calculate rotation quaternion
                axis = np.cross(z_down, gravity_unit)
                axis = axis / (np.linalg.norm(axis) + 1e-10)
                angle = np.arccos(np.clip(np.dot(z_down, gravity_unit), -1, 1))
                
                self.quaternion = np.array([
                    np.cos(angle/2),
                    axis[0] * np.sin(angle/2),
                    axis[1] * np.sin(angle/2),
                    axis[2] * np.sin(angle/2)
                ])
            
            # Accel bias - expect gravity in current orientation
            expected_gravity = self.rotate_vector(np.array([0, 0, -GRAVITY_MAGNITUDE]), 
                                                 self.quaternion_conjugate(self.quaternion))
            self.accel_bias = accel_mean - expected_gravity
            
            # Initialize circular buffers
            for _ in range(LIST_SIZE):
                self.measured_accel.append(accel_mean - self.accel_bias)
                self.measured_gyro.append(gyro_samples[-1] - self.gyro_bias)
            
            self.initialized = True
            print(f"✓ INS initialized (reference-based) with {len(self.init_samples)} samples")
            print(f"  Gyro bias: [{self.gyro_bias[0]:.6f}, {self.gyro_bias[1]:.6f}, {self.gyro_bias[2]:.6f}] rad/s")
            print(f"  Accel bias: [{self.accel_bias[0]:.4f}, {self.accel_bias[1]:.4f}, {self.accel_bias[2]:.4f}] m/s²")
            print(f"  Accel magnitude: {accel_magnitude:.3f} m/s² (gravity: {GRAVITY_MAGNITUDE:.3f})")
            print(f"  Initial quaternion: [{self.quaternion[0]:.4f}, {self.quaternion[1]:.4f}, {self.quaternion[2]:.4f}, {self.quaternion[3]:.4f}]")
            print(f"  Static thresholds: accel={STATIC_ACCEL_THRESHOLD:.4f}, gyro={STATIC_GYRO_THRESHOLD:.6f}")
            
    def process_measurement(self, data: EnhancedIMUData) -> Optional[INSState]:
        """Process measurement based on reference implementation"""
        if not self.initialized:
            self.estimate_bias(data)
            return None
            
        # Calculate time delta
        dt = 1.0 / MEASUREMENT_FREQUENCY  # Default dt
        if self.last_timestamp is not None:
            dt = data.timestamp - self.last_timestamp
            dt = max(min(dt, 0.1), 0.0001)  # Clamp dt to reasonable range
        self.last_timestamp = data.timestamp
        
        # Apply bias correction
        accel_corrected = data.acceleration - self.accel_bias
        gyro_corrected = data.angular_velocity - self.gyro_bias
        
        # Add to circular buffers for Gaussian filtering
        self.measured_accel.append(accel_corrected)
        self.measured_gyro.append(gyro_corrected)
        
        # Apply Gaussian filtering (from reference code)
        if len(self.measured_accel) >= LIST_SIZE:
            accel_filtered = np.array([
                self.apply_gaussian_filter(self.measured_accel, 0),
                self.apply_gaussian_filter(self.measured_accel, 1),
                self.apply_gaussian_filter(self.measured_accel, 2)
            ])
            gyro_filtered = np.array([
                self.apply_gaussian_filter(self.measured_gyro, 0),
                self.apply_gaussian_filter(self.measured_gyro, 1),
                self.apply_gaussian_filter(self.measured_gyro, 2)
            ])
        else:
            accel_filtered = accel_corrected
            gyro_filtered = gyro_corrected
        
        # Update attitude using Madgwick with calibration conditions
        self.madgwick_update_with_conditions(gyro_filtered, accel_filtered, dt)
        
        # Transform acceleration to global frame and remove gravity
        body_accel = accel_filtered - self.current_gravity
        global_accel = self.rotate_vector(body_accel, self.quaternion)
        
        # Update velocity and position
        self.velocity += global_accel * dt
        self.position += self.velocity * dt
        
        # Apply zero velocity correction (from reference code)
        if self.zero_velocity_correction(self.velocity, dt):
            self.velocity = np.array([0.0, 0.0, 0.0])
            print("ZUPT: Velocity reset to zero (static detected)")
        
        self.sample_count += 1
        
        return INSState(
            timestamp=data.timestamp,
            quaternion=self.quaternion.copy(),
            velocity=self.velocity.copy(),
            position=self.position.copy(),
            accel_bias=self.accel_bias.copy(),
            gyro_bias=self.gyro_bias.copy(),
            gravity_vector=self.current_gravity.copy()
        )

# Global variables must be initialized after class definitions  
def cleanup_serial():
    """Clean up serial port resources"""
    global _serial_port
    if _serial_port and _serial_port.is_open:
        print("\nClosing serial port...")
        _serial_port.close()
        _serial_port = None

def signal_handler(signum, frame):
    """Handle SIGINT and SIGTERM signals"""
    print(f"\nReceived signal {signum}, shutting down...")
    print_statistics()
    cleanup_serial()
    sys.exit(0)

def print_statistics():
    """Print processing statistics"""
    elapsed = time.time() - _stats['start_time']
    print(f"\n=== Processing Statistics ===")
    print(f"Runtime: {elapsed:.1f} seconds")
    print(f"Total frames: {_stats['total_frames']}")
    print(f"INS outputs: {_stats['ins_outputs']}")
    print(f"CRC errors: {_stats['crc_errors']}")
    print(f"Sync errors: {_stats['sync_errors']}")
    if elapsed > 0:
        print(f"Frame rate: {_stats['total_frames']/elapsed:.1f} Hz")
        print(f"INS rate: {_stats['ins_outputs']/elapsed:.1f} Hz")
    if _stats['total_frames'] > 0:
        print(f"Error rate: {(_stats['crc_errors'] + _stats['sync_errors'])/_stats['total_frames']*100:.2f}%")

def parse_enhanced_frame(payload: bytes) -> Optional[EnhancedIMUData]:
    """Parse enhanced IMU frame (header 'Z') with detailed debugging"""
    try:
        # Debug: Print raw payload info
        if len(payload) != 42:  # Expected payload size without CRLF
            print(f"DEBUG: Unexpected payload size: {len(payload)} (expected 42)")
            return None
            
        # Debug: Print first few bytes
        header_byte = payload[0:1]
        print(f"DEBUG: Header byte: {header_byte} (expected b'Z')")
        
        unpacked = ENHANCED_STRUCT.unpack(payload)
        header = unpacked[0]
        sec, msec = unpacked[1:3]
        ax, ay, az, gx, gy, gz, temp = unpacked[3:10]
        hw_timestamp = unpacked[10]
        crc_recv = unpacked[11]
        
        print(f"DEBUG: Unpacked header: {header} (type: {type(header)})")
        
        if header != b"Z":
            print(f"DEBUG: Header mismatch - got {header}, expected b'Z'")
            return None
            
        # Verify CRC
        crc_calc = crc8_maxim(payload[:-1])
        if crc_calc != crc_recv:
            _stats['crc_errors'] += 1
            print(f"DEBUG: CRC mismatch - calculated {crc_calc}, received {crc_recv}")
            return None
            
        timestamp = sec + msec / 1000.0
        acceleration = np.array([ax, ay, az])
        angular_velocity = np.array([np.radians(gx), np.radians(gy), np.radians(gz)])  # Convert to rad/s
        
        print(f"DEBUG: Successfully parsed frame - timestamp: {timestamp}")
        
        return EnhancedIMUData(
            timestamp=timestamp,
            hw_timestamp=hw_timestamp,
            acceleration=acceleration,
            angular_velocity=angular_velocity,
            temperature=temp
        )
        
    except struct.error as e:
        print(f"DEBUG: Struct unpack error: {e}")
        print(f"DEBUG: Payload length: {len(payload)}, content: {payload[:10]}...")
        return None
    except Exception as e:
        print(f"DEBUG: Unexpected error in parse_enhanced_frame: {e}")
        return None

def process_frame_data():
    """Main frame processing loop - optimized for performance"""
    global _serial_port, _ins_processor
    
    print("Starting optimized frame processing...")
    frame_count = 0
    detected_format = None
    debug_mode = True
    
    while True:
        # Auto-detect frame format on first run
        if detected_format is None:
            header_byte = _serial_port.read(1)
            if not header_byte:
                continue
                
            if header_byte == b'X':
                detected_format = 'basic'
                frame_size = BASIC_FRAME_SIZE
                if debug_mode:
                    print("✓ Detected BASIC format (X header)")
            elif header_byte == b'Z':
                detected_format = 'enhanced'
                frame_size = ENHANCED_FRAME_SIZE
                if debug_mode:
                    print("✓ Detected ENHANCED format (Z header)")
            else:
                continue
                
            remaining_data = _serial_port.read(frame_size - 1)
            if len(remaining_data) != frame_size - 1:
                detected_format = None
                continue
                
            frame_data = header_byte + remaining_data
        else:
            # Read complete frame of detected format
            frame_size = ENHANCED_FRAME_SIZE if detected_format == 'enhanced' else BASIC_FRAME_SIZE
            frame_data = _serial_port.read(frame_size)
            
        frame_count += 1
        
        if len(frame_data) != frame_size:
            detected_format = None
            continue
            
        payload, crlf = frame_data[:-2], frame_data[-2:]
        
        if crlf != b"\r\n":
            _stats['sync_errors'] += 1
            detected_format = None
            _serial_port.read(1)
            continue
            
        data = parse_frame(payload)
        if data:
            _stats['total_frames'] += 1
            
            # Process with INS
            ins_state = _ins_processor.process_measurement(data)
            
            if ins_state:
                _stats['ins_outputs'] += 1
                
                # Turn off debug mode after first INS output
                if debug_mode:
                    debug_mode = False
                    print("✓ INS active - switching to normal output mode")
                
                # Convert quaternion to Euler for display
                euler = _ins_processor.quaternion_to_euler(ins_state.quaternion)
                
                # Display output (every 64th sample for readability)
                if _stats['ins_outputs'] % 64 == 0:
                    print(f"{ins_state.timestamp:.3f}  "
                          f"acc=({data.acceleration[0]:+.3f},{data.acceleration[1]:+.3f},{data.acceleration[2]:+.3f})  "
                          f"gyro=({np.degrees(data.angular_velocity[0]):+.3f},{np.degrees(data.angular_velocity[1]):+.3f},{np.degrees(data.angular_velocity[2]):+.3f})°/s  "
                          f"rpy=({euler[0]:+.1f},{euler[1]:+.1f},{euler[2]:+.1f})°  "
                          f"vel=({ins_state.velocity[0]:+.3f},{ins_state.velocity[1]:+.3f},{ins_state.velocity[2]:+.3f})  "
                          f"pos=({ins_state.position[0]:+.3f},{ins_state.position[1]:+.3f},{ins_state.position[2]:+.3f})  "
                          f"T={data.temperature:.1f}°C")
            else:
                # During initialization - show progress less frequently
                if _stats['total_frames'] % 50 == 0:
                    progress = len(_ins_processor.init_samples)
                    total = BIAS_ESTIMATION_SAMPLES
                    percent = (progress * 100) // total
                    print(f"INS Init: {percent}% ({progress}/{total}) - Rate: {_stats['total_frames']/((time.time() - _stats['start_time']) or 1):.1f}Hz")
                    
        # Turn off debug after 100 frames regardless
        if frame_count == 100 and debug_mode:
            debug_mode = False
            print("✓ Debug mode disabled after 100 frames")
            
        # Periodic statistics
        if frame_count % 2000 == 0:
            elapsed = time.time() - _stats['start_time']
            print(f"Stats: {frame_count} frames, {_stats['total_frames']} valid, "
                  f"{_stats['ins_outputs']} INS outputs, "
                  f"Rate: {_stats['total_frames']/elapsed:.1f}Hz")

def main() -> None:
    global _serial_port, _ins_processor
    
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    atexit.register(cleanup_serial)
    
    # Initialize INS processor
    filter_type = input("Select filter type (madgwick/mahony/simple) [madgwick]: ").strip().lower()
    if filter_type not in ['madgwick', 'mahony', 'simple']:
        filter_type = 'madgwick'
    
    _ins_processor = AdvancedINSProcessor(filter_type=filter_type)
    
    print(f"=== Advanced Python INS Processor ===")
    print(f"Port: {PORT} @ {BAUDRATE} bps")
    print(f"Filter: {filter_type.capitalize()}")
    print(f"Frame format: Enhanced (Z header, 44 bytes)")
    print("Press Ctrl+C to stop and show statistics\n")
    
    try:
        _serial_port = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
        _serial_port.reset_input_buffer()
        
        _stats['start_time'] = time.time()
        process_frame_data()
                  
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        cleanup_serial()
        sys.exit(1)
    except Exception as e:
        print(f"Unexpected error: {e}")
        cleanup_serial()
        sys.exit(1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped by user")
        print_statistics()
        cleanup_serial()
    except Exception as e:
        print(f"Error: {e}")
        cleanup_serial()
        sys.exit(1)