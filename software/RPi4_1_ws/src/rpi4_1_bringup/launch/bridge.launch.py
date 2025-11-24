import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bringup_dir = get_package_share_directory('rpi4_1_bringup')
    ekf_config = os.path.join(bringup_dir, 'config', 'ekf.yaml')

    return LaunchDescription([
        # 1. micro-ROS Agent (Serial connection to Pico)
        # Node A is connected via USB, so we use the Serial transport.
        # Default device for Pico on Linux is usually /dev/ttyACM0.
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyACM0', '-v6'],
            output='screen'
        ),

        # 2. Actuator Bridge
        Node(
            package='actuator_bridge',
            executable='actuator_bridge_node',
            name='actuator_bridge',
            output='screen',
            parameters=[
                {'dxl_device': '/dev/ttyUSB0'},
                {'dxl_baudrate': 57600}
            ]
        ),

        # 3. Sensor Processing
        Node(
            package='sensor_processing',
            executable='sensor_processing_node',
            name='sensor_processing',
            output='screen',
            parameters=[
                {'imu_sample_rate': 100.0}
            ]
        ),

        # 4. EKF Node (robot_localization)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        )
    ])
