from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    entrance_exploration_pkg_dir = get_package_share_directory('entrance_exploration')
    entrance_exploration_config_file = os.path.join(
        entrance_exploration_pkg_dir,
        'config',
        'entrance_exploration_params_sim.yaml'
    )
    
    delivery_executor_pkg_dir = get_package_share_directory('delivery_executor')
    delivery_executor_config_file = os.path.join(
        delivery_executor_pkg_dir,
        'config',
        'delivery_executor_sim.yaml'
    )
    
    osm_dir = get_package_share_directory('utils_pkg')
    osm_file_path = os.path.join(
        osm_dir,
        'resource',
        'osm',
        'medium.osm'
    )
    
    task_planning_server_node = Node(
        package='task_planning',
        executable='task_planning_server',
        name='task_planning_server',
        parameters=[
            {'osm_file_path': osm_file_path}
        ],
        output='screen',
        emulate_tty=True
    )

    recognition_node = Node(
        package='building_entrance_recognition',
        executable='entrance_recognition_server',
        name='entrance_recognition_server',
        output='screen',
        emulate_tty=True
    )
    
    exploration_node = Node(
        package='entrance_exploration',
        executable='entrance_exploration_action_server',
        parameters=[
            entrance_exploration_config_file,
            {'osm_file_path': osm_file_path}
        ],
        output='screen',
        emulate_tty=True
    )

    delivery_executor_server_node = Node(
        package='delivery_executor',
        executable='delivery_executor_action_server',
        output='screen',
        emulate_tty=True,
        parameters=[
            delivery_executor_config_file,
            {'osm_file_path': osm_file_path}
        ]
    )

    return LaunchDescription([
        task_planning_server_node,
        recognition_node,
        exploration_node,
        TimerAction(
            period=2.0,
            actions=[delivery_executor_server_node]
        )
    ]) 