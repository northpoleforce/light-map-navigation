from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('grounded_sam2'),
        'config',
        'gsam2_params.yaml'
    ])

    current_pythonpath = os.environ.get('PYTHONPATH', '')
    custom_path = '/workspaces/light-map-navigation/src/grounded_sam2/grounded_sam2/grounded_sam2'
    os.environ['PYTHONPATH'] = f"{custom_path}:{current_pythonpath}"

    return LaunchDescription([
        Node(
            package='grounded_sam2',
            executable='gsam2_node',
            name='grounded_sam2_node',
            prefix=['/opt/conda/envs/dl_env/bin/python'],
            output='screen',
            parameters=[config_file]        )
    ])