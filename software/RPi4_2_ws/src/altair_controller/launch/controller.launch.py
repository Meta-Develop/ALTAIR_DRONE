import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    
    # Paths
    # Assuming 'altair_description' is in the workspace or installed
    # If developing in a split workspace without sourcing the other, this might fail
    # But standard ROS 2 practice assumes packages are discoverable.
    
    # We'll try to find altair_description. 
    # If not found, we might default to a relative path if provided, 
    # but Command('xacro ...') needs a file path.
    
    pkg_controller = get_package_share_directory('altair_controller')
    
    # Try to find description package
    try:
        pkg_description = get_package_share_directory('altair_description')
        urdf_file = os.path.join(pkg_description, 'urdf', 'altair.urdf')
    except Exception:
        # Fallback for the specific task context if package not built yet
        # Assuming we are in software/RPi4_2_ws, common is ../common
        # This path is tricky in launch files.
        urdf_file = os.path.expanduser('~/altair_project/software/common/altair_description/urdf/altair.urdf')

    # Arguments
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='PID',
        description='Control Mode: PID or NMPC'
    )

    # Nodes
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['cat ', urdf_file])}] # Simple cat for URDF (not xacro unless needed)
    )

    controller_node = Node(
        package='altair_controller',
        executable='controller_node',
        name='altair_controller',
        output='screen',
        parameters=[{
            'mode': LaunchConfiguration('mode'),
            'urdf_path': urdf_file
        }]
    )

    return LaunchDescription([
        mode_arg,
        rsp_node,
        controller_node
    ])
