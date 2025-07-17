#!/usr/bin/env python3
"""
Example usage of the enhanced Madgwick trajectory system
Demonstrates different configurations and ROS2 integration
"""

import os
import sys
from loguru import logger

# Import the SpresenseIMUProcessor class
sys.path.append(os.path.dirname(__file__))
exec(open('2_madgwick_trajectory.py').read())
# Alternative: from madgwick_trajectory import SpresenseIMUProcessor

def example_basic_usage():
    """Basic usage without ROS2"""
    logger.info("=== Basic Usage Example ===")
    
    processor = SpresenseIMUProcessor(frequency=60.0)
    
    # Run for a short time
    try:
        processor.run_realtime(enable_ros2=False)
    except KeyboardInterrupt:
        logger.info("Basic example completed")

def example_with_zupt_cupt_disabled():
    """Example with ZUPT/CUPT disabled"""
    logger.info("=== ZUPT/CUPT Disabled Example ===")
    
    processor = SpresenseIMUProcessor(
        frequency=60.0,
        zupt_enabled=False,
        cupt_enabled=False
    )
    
    try:
        processor.run_realtime(enable_ros2=False)
    except KeyboardInterrupt:
        logger.info("ZUPT/CUPT disabled example completed")

def example_with_ros2():
    """Example with ROS2 publishing"""
    logger.info("=== ROS2 Publishing Example ===")
    
    processor = SpresenseIMUProcessor(frequency=60.0)
    
    try:
        processor.run_realtime(enable_ros2=True)
    except KeyboardInterrupt:
        logger.info("ROS2 example completed")

def example_custom_callback():
    """Example with custom callback for trajectory data"""
    logger.info("=== Custom Callback Example ===")
    
    def trajectory_callback(quaternion, timestamp, raw_data):
        """Custom callback to process trajectory data"""
        if 'trajectory' in raw_data:
            traj = raw_data['trajectory']
            pos = traj['position']
            vel = traj['velocity']
            
            # Calculate distance from origin
            distance = (pos[0]**2 + pos[1]**2 + pos[2]**2)**0.5
            speed = (vel[0]**2 + vel[1]**2 + vel[2]**2)**0.5
            
            logger.info(f"Distance from origin: {distance:.3f}m, Speed: {speed:.3f}m/s")
            
            # ZUPT/CUPT status
            if traj.get('zupt_active', False):
                logger.warning("ZUPT active - velocity reset")
            if traj.get('cupt_active', False):
                logger.warning("CUPT active - position reset")
    
    processor = SpresenseIMUProcessor(frequency=60.0)
    processor.add_quaternion_callback(trajectory_callback)
    
    try:
        processor.run_realtime(enable_ros2=False)
    except KeyboardInterrupt:
        logger.info("Custom callback example completed")

def show_environment_variables():
    """Show all environment variables that can be used"""
    logger.info("=== Environment Variables ===")
    logger.info("Log Level Control:")
    logger.info("  LOG_LEVEL=DEBUG|INFO|WARNING|ERROR (default: INFO)")
    logger.info("")
    logger.info("ZUPT/CUPT Control:")
    logger.info("  ZUPT_ENABLED=True|False (default: True)")
    logger.info("  CUPT_ENABLED=True|False (default: True)")
    logger.info("  ZUPT_VELOCITY_THRESHOLD=0.1 (default: 0.1 m/s)")
    logger.info("  ZUPT_ACCEL_THRESHOLD=0.5 (default: 0.5 m/s²)")
    logger.info("  CUPT_POSITION_THRESHOLD=10.0 (default: 10.0 m)")
    logger.info("")
    logger.info("ROS2 Control:")
    logger.info("  ENABLE_ROS2=True|False (default: False)")
    logger.info("")
    logger.info("Examples:")
    logger.info("  LOG_LEVEL=DEBUG ZUPT_ENABLED=False python 2_madgwick_trajectory.py")
    logger.info("  ENABLE_ROS2=True python 2_madgwick_trajectory.py")
    logger.info("  LOG_LEVEL=DEBUG ENABLE_ROS2=True python 2_madgwick_trajectory.py")

def main():
    """Main function to run examples"""
    if len(sys.argv) < 2:
        print("Usage: python example_usage.py <example_number>")
        print("Examples:")
        print("  1 - Basic usage")
        print("  2 - ZUPT/CUPT disabled")
        print("  3 - ROS2 publishing")
        print("  4 - Custom callback")
        print("  5 - Show environment variables")
        return
    
    example_num = int(sys.argv[1])
    
    if example_num == 1:
        example_basic_usage()
    elif example_num == 2:
        example_with_zupt_cupt_disabled()
    elif example_num == 3:
        example_with_ros2()
    elif example_num == 4:
        example_custom_callback()
    elif example_num == 5:
        show_environment_variables()
    else:
        logger.error(f"Unknown example number: {example_num}")

if __name__ == "__main__":
    main()