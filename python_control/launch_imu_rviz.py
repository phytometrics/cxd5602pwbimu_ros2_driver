#!/usr/bin/env python3
"""
Launch script for IMU trajectory tracking with RViz2 visualization
"""

import os
import sys
import subprocess
import signal
import time
from pathlib import Path

def signal_handler(signum, frame):
    """Handle Ctrl+C gracefully"""
    print("\nShutting down...")
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    
    # Get the directory of this script
    script_dir = Path(__file__).parent
    
    # Use Mac-compatible config if on macOS
    import platform
    if platform.system() == "Darwin":
        print("Detected macOS, using Mac-specific RViz config")
        rviz_config = script_dir / "rviz_config_mac.rviz"
    else:
        rviz_config = script_dir / "rviz_config.rviz"
    
    print("Starting IMU Trajectory Tracking with RViz2...")
    print("="*50)
    
    # Check if RViz2 is available
    try:
        subprocess.run(["which", "rviz2"], check=True, capture_output=True)
    except subprocess.CalledProcessError:
        print("ERROR: rviz2 not found. Please install ROS2 and ensure it's in your PATH.")
        print("Source your ROS2 setup: source /opt/ros/humble/setup.bash")
        return 1
    
    processes = []
    
    try:
        # Start RViz2 with our configuration
        print("1. Starting RViz2...")
        rviz_cmd = ["rviz2", "-d", str(rviz_config)]
        rviz_process = subprocess.Popen(rviz_cmd)
        processes.append(("RViz2", rviz_process))
        
        # Wait a moment for RViz2 to start
        time.sleep(2)
        
        # Start the IMU trajectory tracker
        print("2. Starting IMU Trajectory Tracker...")
        
        # Determine Python command and script path
        python_cmd = sys.executable
        imu_script = script_dir / "3_ros2.py"
        
        imu_cmd = [python_cmd, str(imu_script), "--ros2"]
        
        print(f"Running: {' '.join(imu_cmd)}")
        imu_process = subprocess.Popen(imu_cmd)
        processes.append(("IMU Tracker", imu_process))
        
        print("\nAll processes started successfully!")
        print("="*50)
        print("RViz2 Topics to monitor:")
        print("  /imu/data         - Raw IMU data")
        print("  /imu/trajectory   - Complete trajectory path")
        print("  /imu/pose         - Current pose")
        print("  /imu/velocity     - Current velocity")
        print("  /imu/acceleration - Current acceleration")
        print("  /imu/zupt_active  - ZUPT status")
        print("  /imu/cupt_active  - CUPT status")
        print("="*50)
        print("Press Ctrl+C to stop all processes")
        
        # Wait for processes to complete
        while True:
            time.sleep(1)
            
            # Check if any process has died
            for name, process in processes:
                if process.poll() is not None:
                    print(f"\n{name} process has terminated")
                    return 1
    
    except KeyboardInterrupt:
        print("\nReceived Ctrl+C, shutting down...")
    
    except Exception as e:
        print(f"Error: {e}")
        return 1
    
    finally:
        # Terminate all processes
        print("Terminating processes...")
        for name, process in processes:
            if process.poll() is None:
                print(f"  Stopping {name}...")
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    print(f"  Force killing {name}...")
                    process.kill()
        
        print("All processes stopped.")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())