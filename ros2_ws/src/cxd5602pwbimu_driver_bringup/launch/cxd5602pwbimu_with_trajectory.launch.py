from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Declare launch arguments
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/ttyUSB0',
        description='Serial device path'
    )
    rviz_config_path = os.path.join(
        get_package_share_directory('cxd5602pwbimu_driver_bringup'),
        'rviz', 'trajectory_visualization.rviz'
    )
    
    # IMU driver node
    imu_driver_node = Node(
        package='cxd5602pwbimu_driver_node',
        executable='cxd5602pwbimu_driver_node_exec',
        name='cxd5602pwbimu_driver_node',
        output='screen',
        parameters=[{'device': LaunchConfiguration('device')}]
    )

    # Madgwick filter node
    madgwick_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick_node',
        output='screen',
        parameters=[{'use_mag': False}]
    )

    # Robot trajectory node
    robot_trajectory_node = Node(
        package='cxd5602pwbimu_driver_bringup',
        executable='robot_trajectory_node',
        name='robot_trajectory_node',
        output='screen',
        parameters=[
            {'velocity_decay': 0.98},
            {'accel_threshold': 0.5},
            {'max_path_length': 2000}
        ]
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        device_arg,
        imu_driver_node,
        madgwick_filter_node,
        robot_trajectory_node,
        rviz_node
    ])