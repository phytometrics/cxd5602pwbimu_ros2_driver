from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
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
        parameters=[{'device': '/dev/ttyUSB0'}]
    )

    # Madgwick filter node
    madgwick_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick_node',
        output='screen',
        parameters=[{'use_mag': False}]
    )

    # Orientation trajectory node
    orientation_trajectory_node = Node(
        package='cxd5602pwbimu_driver_bringup',
        executable='orientation_trajectory_node',
        name='orientation_trajectory_node',
        output='screen',
        parameters=[
            {'movement_scale': 10.0},
            {'max_path_length': 1000}
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
        imu_driver_node,
        madgwick_filter_node,
        orientation_trajectory_node,
        rviz_node
    ])