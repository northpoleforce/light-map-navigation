from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('entrance_exploration')
    
    # Path to the configuration file
    config_file = os.path.join(
        pkg_dir,
        'config',
        'entrance_exploration_params_sim.yaml'
    )
    
    # Create and return launch description
    return LaunchDescription([
        Node(
            package='entrance_exploration',
            executable='entrance_exploration_action_server',
            name='entrance_exploration_action_server',
            parameters=[config_file],
            output='screen'
        )
    ])
