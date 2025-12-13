import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch import actions
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
    
    
    # Path to CycloneDDS config (in project root)
    # Windows path adjustment might be needed if running partially in WSL or native
    # Assuming this runs where 'd:\home\...' is mounted or accessible.
    # However, strictly speaking, we want a portable way.
    # If the user is running this from 'software/gcs_tools_ws', the root is 2 levels up.
    
    # Let's find the project root dynamically or hardcode relative to WS
    # For now, let's assume the user runs it from the workspace root or we use a fixed path structure
    # Given the user context "d:\home\6.kennkyuu\ALTAIR_DRONE", let's try to map it.
    
    # Better approach: The user runs this on WINDOWS.
    # The file is at d:\home\6.kennkyuu\ALTAIR_DRONE\cyclonedds_pc.xml
    cyclonedds_config = "d:\\home\\6.kennkyuu\\ALTAIR_DRONE\\cyclonedds_pc.xml"

    return LaunchDescription([
        # Force CycloneDDS Config
        actions.SetEnvironmentVariable('CYCLONEDDS_URI', 'file:///' + cyclonedds_config),

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
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_file]
        # ),
        Node(
            package='system_monitor',
            executable='watchdog',
            name='system_monitor',
            output='screen'
        )
    ])