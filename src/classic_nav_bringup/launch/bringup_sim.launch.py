import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction, Shutdown
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals, IfCondition

def generate_launch_description():
    # Get the launch directory
    classic_nav_bringup_dir = get_package_share_directory('classic_nav_bringup')
    pb_gazebo_simulation_launch_dir = os.path.join(get_package_share_directory('gazebo_simulation'), 'launch')
    navigation2_launch_dir = os.path.join(get_package_share_directory('classic_navigation'), 'launch')

    # Create the launch configuration variables
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_lio_rviz = LaunchConfiguration('lio_rviz')
    use_nav_rviz = LaunchConfiguration('nav_rviz')

    ################################ robot_description parameters start ###############################
    launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('classic_nav_bringup'), 'config', 'simulation', 'measurement_params_sim.yaml')))
    robot_description = Command(['xacro ', os.path.join(
    get_package_share_directory('classic_nav_bringup'), 'urdf', 'sentry_robot_sim.xacro'),
    ' xyz:=', launch_params['base_link2livox_frame']['xyz'], ' rpy:=', launch_params['base_link2livox_frame']['rpy']])
    ################################# robot_description parameters end ################################

    ########################## linefit_ground_segementation parameters start ##########################
    segmentation_params = os.path.join(classic_nav_bringup_dir, 'config', 'simulation', 'segmentation_sim.yaml')
    ########################## linefit_ground_segementation parameters end ############################

    #################################### FAST_LIO parameters start ####################################
    fastlio_mid360_params = os.path.join(classic_nav_bringup_dir, 'config', 'simulation', 'fastlio_mid360_sim.yaml')
    fastlio_rviz_cfg_dir = os.path.join(classic_nav_bringup_dir, 'rviz', 'fastlio.rviz')
    ##################################### FAST_LIO parameters end #####################################

    ################################### POINT_LIO parameters start ####################################
    pointlio_mid360_params = os.path.join(classic_nav_bringup_dir, 'config', 'simulation', 'pointlio_mid360_sim.yaml')
    pointlio_rviz_cfg_dir = os.path.join(classic_nav_bringup_dir, 'rviz', 'pointlio.rviz')
    #################################### POINT_LIO parameters end #####################################

    ################################## slam_toolbox parameters start ##################################
    slam_toolbox_map_dir = PathJoinSubstitution([classic_nav_bringup_dir, 'map', world])
    slam_toolbox_localization_file_dir = os.path.join(classic_nav_bringup_dir, 'config', 'simulation', 'mapper_params_localization_sim.yaml')
    slam_toolbox_mapping_file_dir = os.path.join(classic_nav_bringup_dir, 'config', 'simulation', 'mapper_params_online_async_sim.yaml')
    ################################### slam_toolbox parameters end ###################################

    ################################### navigation2 parameters start ##################################
    nav2_map_dir = PathJoinSubstitution([classic_nav_bringup_dir, 'map', world]), ".yaml"
    nav2_params_file_dir = os.path.join(classic_nav_bringup_dir, 'config', 'simulation', 'nav2_params_sim.yaml')
    ################################### navigation2 parameters end ####################################

    ################################ icp_registration parameters start ################################
    icp_pcd_dir = PathJoinSubstitution([classic_nav_bringup_dir, 'PCD', world]), ".pcd"
    icp_registration_params_dir = os.path.join(classic_nav_bringup_dir, 'config', 'simulation', 'icp_registration_sim.yaml')
    ################################# icp_registration parameters end #################################

    ############################# pointcloud_downsampling parameters start ############################
    pointcloud_downsampling_config_dir = os.path.join(classic_nav_bringup_dir, 'config', 'simulation', 'pointcloud_downsampling_sim.yaml')
    ############################# pointcloud_downsampling parameters start ############################

    # Declare launch options
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_use_lio_rviz_cmd = DeclareLaunchArgument(
        'lio_rviz',
        default_value='False',
        description='Visualize FAST_LIO or Point_LIO cloud_map if true')

    declare_nav_rviz_cmd = DeclareLaunchArgument(
        'nav_rviz',
        default_value='True',
        description='Visualize navigation2 if true')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='LARGE_OSM',
        description='Select world (map file, pcd file, world file share the same name prefix as the this parameter)')

    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='',
        description='Choose mode: nav, mapping')

    declare_localization_cmd = DeclareLaunchArgument(
        'localization',
        default_value='',
        description='Choose localization method: slam_toolbox, amcl, icp')

    declare_LIO_cmd = DeclareLaunchArgument(
        'lio',
        default_value='fast_lio',
        description='Choose lio alogrithm: fastlio or pointlio')

    # Specify the actions
    start_gazebo_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pb_gazebo_simulation_launch_dir, 'gazebo_simulation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'robot_description': robot_description,
            'rviz': 'False'}.items()
    )

    bringup_imu_complementary_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[
            {'do_bias_estimation': True},
            {'do_adaptive_gain': True},
            {'use_mag': False},
            {'gain_acc': 0.01},
            {'gain_mag': 0.01},
        ],
        remappings=[
            ('/imu/data_raw', '/livox/imu'),
        ],
        on_exit=Shutdown()
    )

    bringup_linefit_ground_segmentation_node = Node(
        package='linefit_ground_segmentation_ros',
        executable='ground_segmentation_node',
        output='screen',
        parameters=[segmentation_params],
        on_exit=Shutdown()
    )

    bringup_pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in',  ['/segmentation/obstacle']),
                    ('scan',  ['/scan'])],
        parameters=[{
            'target_frame': 'livox_frame',
            'transform_tolerance': 0.01,
            'min_height': -1.0,
            'max_height': 0.1,
            'angle_min': -3.14159,  # -M_PI/2
            'angle_max': 3.14159,   # M_PI/2
            'angle_increment': 0.0043,  # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan',
        on_exit=Shutdown()
    )

    bringup_LIO_group = GroupAction([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                # Useless arguments, provided by LIO in publish_odometry() function
                # '--x', '0.0',
                # '--y', '0.0',
                # '--z', '0.0',
                # '--roll', '0.0',
                # '--pitch', '0.0',
                # '--yaw', '0.0',
                '--frame-id', 'odom',
                '--child-frame-id', 'lidar_odom'
            ],
            on_exit=Shutdown()
        ),

        GroupAction(
            condition = LaunchConfigurationEquals('lio', 'fastlio'),
            actions=[
            Node(
                package='fast_lio',
                executable='fastlio_mapping',
                parameters=[
                    fastlio_mid360_params,
                    {use_sim_time: use_sim_time}
                ],
                output='screen',
                on_exit=Shutdown()
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', fastlio_rviz_cfg_dir],
                condition = IfCondition(use_lio_rviz),
                on_exit=Shutdown()
            ),
        ]),

        GroupAction(
            condition = LaunchConfigurationEquals('lio', 'pointlio'),
            actions=[
            Node(
                package='point_lio',
                executable='pointlio_mapping',
                name='laserMapping',
                output='screen',
                parameters=[
                    pointlio_mid360_params,
                    {'use_sim_time': use_sim_time,
                    'use_imu_as_input': False,  # Change to True to use IMU as input of Point-LIO
                    'prop_at_freq_of_imu': True,
                    'check_satu': False,
                    'init_map_size': 10,
                    'point_filter_num': 3,  # Options: 1, 3
                    'space_down_sample': True,
                    'filter_size_surf': 0.5,  # Options: 0.5, 0.3, 0.2, 0.15, 0.1
                    'filter_size_map': 0.5,  # Options: 0.5, 0.3, 0.15, 0.1
                    'ivox_nearby_type': 26,   # Options: 0, 6, 18, 26
                    'runtime_pos_log_enable': False}
                ],
                on_exit=Shutdown()
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', pointlio_rviz_cfg_dir],
                condition = IfCondition(use_lio_rviz),
                on_exit=Shutdown()
            )
        ])
    ])

    start_localization_group = GroupAction(
        condition = LaunchConfigurationEquals('mode', 'nav'),
        actions=[
            Node(
                condition = LaunchConfigurationEquals('localization', 'slam_toolbox'),
                package='slam_toolbox',
                executable='localization_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[
                    slam_toolbox_localization_file_dir,
                    {'use_sim_time': use_sim_time,
                    'map_file_name': slam_toolbox_map_dir,
                    'map_start_pose': [0.0, 0.0, 0.0]}
                ],
                on_exit=Shutdown()
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir,'localization_amcl_launch.py')),
                condition = LaunchConfigurationEquals('localization', 'amcl'),
                launch_arguments = {
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params_file_dir}.items()
            ),

            TimerAction(
                period=7.0,
                actions=[
                    Node(
                        condition=LaunchConfigurationEquals('localization', 'icp'),
                        package='icp_registration',
                        executable='icp_registration_node',
                        output='screen',
                        parameters=[
                            icp_registration_params_dir,
                            {'use_sim_time': use_sim_time,
                                'pcd_path': icp_pcd_dir}
                        ],
                        on_exit=Shutdown()
                        # arguments=['--ros-args', '--log-level', ['icp_registration:=', 'DEBUG']]
                    )
                ]
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'map_server_launch.py')),
                condition = LaunchConfigurationNotEquals('localization', 'slam_toolbox'),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': nav2_map_dir,
                    'params_file': nav2_params_file_dir,
                    'container_name': 'nav2_container'}.items())
        ]
    )

    start_mapping = Node(
        condition = LaunchConfigurationEquals('mode', 'mapping'),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_toolbox_mapping_file_dir,
            {'use_sim_time': use_sim_time,}
        ],
        on_exit=Shutdown()
    )

    start_navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'bringup_classic_navigation.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': nav2_map_dir,
            'params_file': nav2_params_file_dir,
            'nav_rviz': use_nav_rviz}.items()
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_lio_rviz_cmd)
    ld.add_action(declare_nav_rviz_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_localization_cmd)
    ld.add_action(declare_LIO_cmd)

    ld.add_action(start_gazebo_simulation)
    ld.add_action(bringup_imu_complementary_filter_node)
    ld.add_action(bringup_linefit_ground_segmentation_node)
    ld.add_action(bringup_pointcloud_to_laserscan_node)
    ld.add_action(bringup_LIO_group)
    ld.add_action(start_localization_group)
    ld.add_action(start_mapping)
    ld.add_action(start_navigation2)

    return ld
