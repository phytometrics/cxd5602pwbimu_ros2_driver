from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch arguments
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/tty.usbserial-1130',
        description='Serial device for IMU'
    )
    
    alpha_arg = DeclareLaunchArgument(
        'alpha',
        default_value='0.98',
        description='Complementary filter alpha parameter'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='imu_link',
        description='Base frame for IMU'
    )
    
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame'
    )

    # IMU driver node
    imu_driver_node = Node(
        package='cxd5602pwbimu_driver_node',
        executable='cxd5602pwbimu_driver_node_exec',
        name='cxd5602pwbimu_driver_node',
        parameters=[{
            'dev': LaunchConfiguration('device')
        }],
        output='screen'
    )

    # IMU trajectory filter node
    imu_filter_node = Node(
        package='imu_trajectory_filter',
        executable='imu_trajectory_filter_node',
        name='imu_trajectory_filter',
        parameters=[{
            'alpha': LaunchConfiguration('alpha'),
            'base_frame': LaunchConfiguration('base_frame'),
            'odom_frame': LaunchConfiguration('odom_frame')
        }],
        output='screen'
    )

    return LaunchDescription([
        device_arg,
        alpha_arg,
        base_frame_arg,
        odom_frame_arg,
        imu_driver_node,
        imu_filter_node
    ])