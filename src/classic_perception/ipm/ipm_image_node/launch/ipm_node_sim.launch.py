from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('ipm_image_node')
    
    # Declare parameter file path argument
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'ipm_params.yaml'),
        description='Full path to the IPM node parameter file'
    )

    # Create IPM node
    ipm_node = Node(
        package='ipm_image_node',
        executable='ipm',
        name='ipm_image_node',
        parameters=[LaunchConfiguration('params_file')],
        output='screen'
    )

    return LaunchDescription([
        params_file_arg,
        ipm_node
    ])