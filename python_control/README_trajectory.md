# IMU Trajectory Tracking with ZUPT/CUPT and ROS2 Integration

Enhanced IMU processing system with trajectory calculation, error suppression, and ROS2 publishing capabilities.

## Features

- **Madgwick filter** for orientation estimation
- **Trajectory calculation** with numerical integration
- **ZUPT (Zero Velocity Update)** to suppress velocity drift
- **CUPT (Coordinate Update)** to suppress position drift  
- **ROS2 publishing** for real-time data streaming
- **Configurable logging** with loguru
- **Environment variable control** for all parameters

## Quick Start

### Basic Usage (without ROS2)
```bash
python 2_madgwick_trajectory.py
```

### With ROS2 Publishing
```bash
ENABLE_ROS2=True python 2_madgwick_trajectory.py
```

### With Debug Logging
```bash
LOG_LEVEL=DEBUG python 2_madgwick_trajectory.py
```

## Environment Variables

### Log Level Control
- `LOG_LEVEL=DEBUG|INFO|WARNING|ERROR` (default: INFO)

### ZUPT/CUPT Control
- `ZUPT_ENABLED=True|False` (default: True)
- `CUPT_ENABLED=True|False` (default: True)
- `ZUPT_VELOCITY_THRESHOLD=0.1` (default: 0.1 m/s)
- `ZUPT_ACCEL_THRESHOLD=0.5` (default: 0.5 m/s²)
- `CUPT_POSITION_THRESHOLD=10.0` (default: 10.0 m)

### ROS2 Control
- `ENABLE_ROS2=True|False` (default: False)

## ROS2 Topics Published

When ROS2 is enabled, the following topics are published:

- `/imu/data` (sensor_msgs/Imu) - Raw IMU data with orientation
- `/imu/pose` (geometry_msgs/PoseStamped) - Current pose
- `/imu/velocity` (geometry_msgs/TwistStamped) - Current velocity
- `/imu/acceleration` (geometry_msgs/AccelStamped) - Current acceleration
- `/imu/trajectory` (nav_msgs/Path) - Complete trajectory path
- `/imu/zupt_active` (std_msgs/Bool) - ZUPT activation status
- `/imu/cupt_active` (std_msgs/Bool) - CUPT activation status

## ZUPT (Zero Velocity Update)

ZUPT detects when the device is stationary and resets velocity to zero to prevent drift accumulation.

**Detection criteria:**
- Average velocity < threshold over last 5 samples
- Average acceleration < threshold over last 5 samples
- Must meet criteria for 3 consecutive times

**Benefits:**
- Reduces velocity drift during stationary periods
- Improves trajectory accuracy for stop-and-go motion

## CUPT (Coordinate Update)

CUPT detects when the device has drifted too far from its starting position and resets to origin.

**Detection criteria:**
- Distance from initial position > threshold

**Benefits:**
- Prevents unbounded position drift
- Useful for applications where device returns to origin

## Example Configurations

### Disable Error Suppression
```bash
ZUPT_ENABLED=False CUPT_ENABLED=False python 2_madgwick_trajectory.py
```

### Sensitive ZUPT Settings
```bash
ZUPT_VELOCITY_THRESHOLD=0.05 ZUPT_ACCEL_THRESHOLD=0.2 python 2_madgwick_trajectory.py
```

### Debug with ROS2
```bash
LOG_LEVEL=DEBUG ENABLE_ROS2=True python 2_madgwick_trajectory.py
```

## Files

- `2_madgwick_trajectory.py` - Main IMU processor with trajectory calculation
- `ros2_imu_publisher.py` - Separate ROS2 publisher class
- `utils/imu.py` - Low-level IMU communication (updated with loguru)
- `example_usage.py` - Usage examples and demonstrations

## Integration Example

```python
from 2_madgwick_trajectory import SpresenseIMUProcessor

# Create processor with custom settings
processor = SpresenseIMUProcessor(
    frequency=60.0,
    zupt_enabled=True,
    cupt_enabled=True
)

# Add custom callback for trajectory data
def my_callback(quaternion, timestamp, raw_data):
    if 'trajectory' in raw_data:
        pos = raw_data['trajectory']['position']
        print(f"Position: {pos}")

processor.add_quaternion_callback(my_callback)

# Run with ROS2
processor.run_realtime(enable_ros2=True)
```

## Troubleshooting

### ROS2 Import Errors
If you get import errors for ROS2:
```bash
# Install ROS2 Python packages
pip install rclpy sensor_msgs geometry_msgs nav_msgs std_msgs

# Or disable ROS2
ENABLE_ROS2=False python 2_madgwick_trajectory.py
```

### Loguru Import Errors
```bash
pip install loguru
```

### Serial Port Issues
Update the PORT variable in the script:
```python
PORT = "/dev/cu.usbserial-110"  # macOS
PORT = "/dev/ttyUSB0"          # Linux
```