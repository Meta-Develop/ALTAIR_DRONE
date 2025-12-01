from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Micro-ROS Agent (Serial)
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyACM0', '-b', '115200'],
            output='screen'
        ),

        # Actuator Bridge
        Node(
            package='actuator_bridge',
            executable='actuator_bridge_node',
            name='actuator_bridge',
            output='screen'
        ),

        # Sensor Processing (Notch Filter)
        Node(
            package='sensor_processing',
            executable='notch_filter_node',
            name='notch_filter',
            output='screen'
        ),

        # EKF (Robot Localization) - Placeholder config
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{'use_sim_time': False}] # Add config file later
        )
    ])
