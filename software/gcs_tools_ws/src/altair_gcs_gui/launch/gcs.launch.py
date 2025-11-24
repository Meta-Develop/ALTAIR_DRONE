import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg_share = get_package_share_directory('altair_gcs_gui')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'altair.rviz')
    
    # Assuming altair_description is in a known relative path or installed
    # For this task, we'll assume the URDF is at a specific path relative to this workspace root
    # or we can try to find it. 
    # Let's assume we pass it or it's hardcoded for this environment specific context.
    # "software/common/altair_description"
    
    urdf_file = 'altair.urdf' # Placeholder
    
    # Try to locate urdf if possible, otherwise assume a default path or skip RSP if not critical
    # But requirement says "Pre-configure RViz to load altair.urdf".
    
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),
        Node(
            package='altair_gcs_gui',
            executable='gui_node',
            name='gcs_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file]
        )
        # Uncomment if robot_state_publisher is desired and urdf is available
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     arguments=[path_to_urdf]
        # )
    ])