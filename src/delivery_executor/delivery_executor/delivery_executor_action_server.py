import rclpy
from rclpy.node import Node
from rclpy.action import (
    ActionServer,
    ActionClient,
    CancelResponse
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from custom_interfaces.action import (
    DeliveryTask,
    EntranceExploration
)
from custom_interfaces.msg import (
    DeliveryFeedback,
    DeliveryResult
)
from custom_interfaces.srv import GetTaskSequence

from nav2_simple_commander.robot_navigator import (
    BasicNavigator,
    TaskResult
)

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PoseStamped

from utils_pkg import (
    OSMHandler,
    CoordinateTransformer,
    OsmGlobalPlanner
)

import math
import sys
import time


class DeliveryExecutorActionServer(Node):
    """
    Action server node for handling delivery tasks.
    
    This class implements a ROS2 action server that manages the execution of delivery tasks.
    It coordinates between navigation, exploration, and task planning components to complete
    delivery missions.
    """
    
    def __init__(self):
        """
        Initialize the DeliveryExecutorActionServer.
        
        Sets up:
        - Core ROS2 node components
        - Action server interface
        - Client connections
        - Map handling system
        - Transform listeners
        
        Raises:
            Exception: If critical components fail to initialize
        """
        super().__init__('delivery_executor_action_server')
        
        try:
            self._init_components()
            self._init_clients()
            self.get_logger().info('Delivery Executor Action Server started successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize action server: {str(e)}')
            raise

    def _init_components(self):
        """
        Initialize core components: action server, OSM handler, and TF listener.
        Sets up the basic infrastructure needed for the delivery executor.
        """
        callback_group = ReentrantCallbackGroup()
        
        # Action server setup
        self._action_server = ActionServer(
            self,
            DeliveryTask,
            'execute_delivery',
            self.execute_callback,
            callback_group=callback_group,
            cancel_callback=self.cancel_callback
        )
        self.get_logger().info('Action server initialized successfully')
        
        # Map and transformation setup
        self._init_map_handler()
        self._init_tf_listener()

    def _init_map_handler(self):
        """
        Initialize OSM map handler and load map data.
        Handles map loading and processing for navigation.
        """
        try:
            self.osm_handler = OSMHandler()
            self.osm_handler.apply_file("medium.osm")
            self.get_logger().info('OSM map loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize map handler: {str(e)}')
            raise

    def _init_tf_listener(self):
        """
        Initialize Transform listener for robot pose tracking.
        Sets up TF buffer and listener for coordinate transformations.
        """
        try:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.get_logger().info('TF listener initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize TF listener: {str(e)}')
            raise

    def _init_clients(self):
        """
        Initialize all client connections for task planning, navigation, and exploration.
        Sets up necessary client interfaces for system operation.
        """
        try:
            self._init_task_planning_client()
            self._init_navigation_client()
            self._init_exploration_client()
            self.get_logger().info('All clients initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize clients: {str(e)}')
            raise

    async def _get_target_coordinates(self, task_info: str) -> tuple:
        """
        Get coordinates for navigation based on task information
        
        Args:
            task_info: String containing building/unit information
            
        Returns:
            tuple: (longitude, latitude) coordinates
            
        Raises:
            Exception: If building or unit cannot be found
        """
        self.get_logger().info(f'Getting target coordinates for task: {task_info}')
        
        # Normalize input string
        info = task_info.lower().replace(" ", "")
        
        try:
            # Handle unit format (building:unit)
            if ':' in info:
                return await self._get_unit_coordinates(info)
            
            # Handle building format
            return await self._get_building_coordinates(info)
            
        except Exception as e:
            self.get_logger().error(f'Failed to get coordinates: {str(e)}')
            raise

    async def _get_unit_coordinates(self, info: str) -> tuple:
        """
        Get coordinates for a specific unit
        
        Args:
            info: String in format 'building_id:unit_id'
            
        Returns:
            tuple: (longitude, latitude) coordinates
            
        Raises:
            Exception: If unit cannot be found
        """
        building_id, unit_id = info.split(':')
        self.get_logger().info(f'Searching for unit location: {unit_id} in building {building_id}')
        
        unit_locations = self.osm_handler.get_node_location_by_name(unit_id)
        if not unit_locations:
            raise Exception(f"Unit not found: {unit_id}")
            
        lat, lon = unit_locations[0]
        self.get_logger().info(f'Found unit coordinates: lat={lat}, lon={lon}')
        return (lon, lat)

    async def _get_building_coordinates(self, info: str) -> tuple:
        """
        Get coordinates for a building center
        
        Args:
            info: Building identifier
            
        Returns:
            tuple: (longitude, latitude) coordinates
            
        Raises:
            Exception: If building cannot be found
        """
        building_centers = self.osm_handler.get_way_center_by_name(info)
        if not building_centers:
            raise Exception(f"Building not found: {info}")
            
        lat, lon = building_centers[0]
        return (lon, lat)

    def _init_task_planning_client(self):
        """
        Initialize the task planning service client.
        Creates client for task sequence generation service.
        """
        self.task_planning_client = self.create_client(
            GetTaskSequence,
            'task_planning'
        )
        
    async def _wait_for_task_planning_service(self):
        """
        Wait for the task planning service to become available.
        Implements exponential backoff for service availability check.
        """
        wait_time = 1.0
        max_wait_time = 10.0
        while not self.task_planning_client.wait_for_service(timeout_sec=wait_time):
            if wait_time > max_wait_time:
                raise TimeoutError("Task planning service not available after maximum wait time")
            self.get_logger().info('Task planning service not available, waiting...')
            wait_time = min(wait_time * 2, max_wait_time)

    async def _get_task_sequence(self, task_description: str):
        """
        Get task sequence from planning service
        
        Args:
            task_description: Natural language description of the task
            
        Returns:
            List of TaskDescription or None if planning fails
            
        Raises:
            TimeoutError: If service is not available after maximum wait time
            Exception: If service call fails
        """
        try:
            await self._wait_for_task_planning_service()
            
            request = GetTaskSequence.Request()
            request.task_description = task_description
            
            self.get_logger().info(f'Requesting task sequence for: {task_description}')
            future = await self.task_planning_client.call_async(request)
            
            if future is not None:
                self.get_logger().info('Successfully received task sequence')
                return future.task_sequence
            else:
                raise Exception('Failed to get task sequence: Empty response')
                
        except TimeoutError as e:
            self.get_logger().error(f'Service timeout: {str(e)}')
            raise
        except Exception as e:
            self.get_logger().error(f'Error getting task sequence: {str(e)}')
            raise

    def _init_navigation_client(self):
        """
        Initialize the navigation client with basic configuration
        
        This method sets up the BasicNavigator instance and configures
        any necessary navigation parameters.
        """
        try:
            self.navigator = BasicNavigator()
            self._configure_navigation_parameters()
            self.get_logger().info('Navigation client initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize navigation client: {str(e)}')
            raise

    def _configure_navigation_parameters(self):
        """
        Configure navigation parameters for optimal performance
        
        Sets up parameters such as path planning behavior, recovery actions,
        and other navigation-specific configurations.
        """
        try:
            # TODO: Add specific navigation parameter configurations
            # self.navigator.setParam('example_param', value)
            pass
        except Exception as e:
            self.get_logger().error(f'Failed to configure navigation parameters: {str(e)}')
            raise

    async def _get_robot_position(self) -> tuple:
        """
        Get current robot position from TF
        
        Returns:
            tuple: (x, y) position in UTM coordinates, or None if position cannot be obtained
            
        Raises:
            TransformException: If transform lookup fails
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time())
            return (transform.transform.translation.x, 
                   transform.transform.translation.y)
        except TransformException as ex:
            self.get_logger().error(f'Could not get robot position: {ex}')
            return None

    async def _navigate_to_pose(self, lon: float, lat: float, goal_handle, feedback, feedback_msg) -> bool:
        """
        Navigate to target position with feedback
        
        Args:
            lon: Target longitude
            lat: Target latitude
            goal_handle: Goal handle for managing task execution
            feedback: Feedback message object
            feedback_msg: Feedback message content
            
        Returns:
            bool: True if navigation succeeded, False otherwise
        """
        self.get_logger().info(f'Starting navigation to coordinates: lon={lon}, lat={lat}')
        try:
            robot_position = await self._get_robot_position()
            if not self._validate_robot_position(robot_position):
                return False

            waypoints = await self._get_navigation_waypoints(robot_position, lon, lat)
            if not waypoints:
                self.get_logger().error('Failed to get navigation waypoints')
                return False
            
            self.get_logger().info(f'Starting waypoint navigation with {len(waypoints)} waypoints')
            return await self._execute_waypoint_navigation(waypoints, robot_position, goal_handle, feedback, feedback_msg)

        except Exception as e:
            self.get_logger().error(f'Navigation failed: {str(e)}')
            return False

    def _validate_robot_position(self, position: tuple) -> bool:
        """
        Validate robot position data
        
        Args:
            position: Tuple containing robot position coordinates
            
        Returns:
            bool: True if position is valid, False otherwise
        """
        if position is None:
            self.get_logger().error('Invalid robot position: None')
            return False
        if not isinstance(position, tuple) or len(position) != 2:
            self.get_logger().error('Invalid robot position format')
            return False
        if not all(isinstance(x, (int, float)) for x in position):
            self.get_logger().error('Invalid robot position coordinates')
            return False
        return True

    async def _get_navigation_waypoints(self, robot_position: tuple, target_lon: float, target_lat: float):
        """
        Get waypoints for navigation path using OSRM
        
        Args:
            robot_position: Current robot position (x, y) in UTM coordinates
            target_lon: Target longitude
            target_lat: Target latitude
            
        Returns:
            list: Navigation waypoints or None if planning fails
        """
        try:
            # Get UTM EPSG code for coordinate transformation
            utm_epsg = CoordinateTransformer.get_utm_epsg(target_lat, target_lon)
            utm_epsg = 32633  # TODO: remove hardcode
            
            # Convert robot position to WGS84
            curr_robot_lon, curr_robot_lat = CoordinateTransformer.utm_to_wgs84(
                robot_position[0], robot_position[1], utm_epsg)
            
            # Format coordinates for OSRM request
            start_pos = f"{curr_robot_lon:.9f},{curr_robot_lat:.9f}"
            target_pos = f"{target_lon:.9f},{target_lat:.9f}"
            
            # Get route from OSRM
            planner = OsmGlobalPlanner("http://101.200.33.217:30457/route/v1/driving/")
            waypoints = planner.get_route(start_pos, target_pos)
            
            if not waypoints:
                self.get_logger().error('Failed to get navigation waypoints')
                return None
                
            return waypoints
            
        except Exception as e:
            self.get_logger().error(f'Error getting waypoints: {str(e)}')
            return None

    async def _execute_waypoint_navigation(self, waypoints, robot_position: tuple, goal_handle, feedback, feedback_msg) -> bool:
        """
        Execute navigation through waypoints with feedback
        
        Args:
            waypoints: List of navigation waypoints
            robot_position: Current robot position (x, y)
            goal_handle: Goal handle for managing task execution
            feedback: Feedback message object
            feedback_msg: Feedback message content
            
        Returns:
            bool: True if navigation succeeded, False otherwise
        """
        try:
            total_waypoints = len(waypoints)
            for i, waypoint in enumerate(waypoints):
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Navigation cancelled')
                    return False

                self._update_task_progress(
                    goal_handle,
                    feedback,
                    feedback_msg,
                    f"Navigating to waypoint {i+1}/{total_waypoints}"
                )
                
                utm_east, utm_north, _ = CoordinateTransformer.wgs84_to_utm(
                    waypoint.lon, waypoint.lat)
                
                pose = self._create_navigation_pose(
                    utm_east, utm_north,
                    self._calculate_waypoint_orientation(
                        i, waypoints, utm_east, utm_north, robot_position
                    )
                )
                
                if not await self._navigate_to_waypoint(pose, i+1, total_waypoints, goal_handle, feedback, feedback_msg):
                    return False
                    
            return True
                
        except Exception as e:
            self.get_logger().error(f'Waypoint navigation error: {str(e)}')
            return False

    def _create_navigation_pose(self, x: float, y: float, quaternion: tuple) -> PoseStamped:
        """
        Create PoseStamped message for navigation
        
        Args:
            x: X coordinate in map frame
            y: Y coordinate in map frame
            quaternion: Orientation as (x, y, z, w) quaternion
            
        Returns:
            PoseStamped: Configured pose message for navigation
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        return pose

    async def _navigate_to_waypoint(self, pose: PoseStamped, current_waypoint: int, 
                                  total_waypoints: int, goal_handle, feedback, feedback_msg) -> bool:
        """
        Navigate to a single waypoint with detailed feedback
        
        Args:
            pose: Target pose for navigation
            current_waypoint: Current waypoint number
            total_waypoints: Total number of waypoints
            goal_handle: Goal handle for managing task execution
            feedback: Feedback message object
            feedback_msg: Feedback message content
            
        Returns:
            bool: True if navigation succeeded, False otherwise
        """
        self.navigator.goToPose(pose)
        
        while not self.navigator.isTaskComplete():
            if goal_handle.is_cancel_requested:
                self.navigator.cancelTask()
                return False
            
            nav_feedback = self.navigator.getFeedback()
            if nav_feedback:
                status_msg = (
                    f"Waypoint {current_waypoint}/{total_waypoints}\n"
                    f"Distance remaining: {nav_feedback.distance_remaining:.2f}m\n"
                )
                
                self._update_task_progress(
                    goal_handle,
                    feedback,
                    feedback_msg,
                    status_msg
                )
                
            time.sleep(2)
            
        result = self.navigator.getResult()
        if result != TaskResult.SUCCEEDED:
            error_msg = f'Navigation to waypoint {current_waypoint}/{total_waypoints} failed: {result}'
            self._update_task_progress(goal_handle, feedback, feedback_msg, error_msg)
            return False
            
        success_msg = f'Successfully reached waypoint {current_waypoint}/{total_waypoints}'
        self._update_task_progress(goal_handle, feedback, feedback_msg, success_msg)
        return True

    def _yaw_to_quaternion(self, yaw: float) -> tuple:
        """
        Convert yaw angle to quaternion
        
        Args:
            yaw: Yaw angle in radians
            
        Returns:
            tuple: (x, y, z, w) quaternion representing the rotation
        """
        return (
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0)
        )

    def _init_exploration_client(self):
        """
        Initialize the entrance exploration action client.
        Sets up client for handling entrance exploration tasks.
        """
        self._exploration_client = ActionClient(
            self,
            EntranceExploration,
            'explore_entrance'
        )
        self.get_logger().info('Exploration client initialized')

    async def _execute_exploration(self, building_id: str, unit_id: str) -> bool:
        """
        Execute entrance exploration task
        
        Args:
            building_id: Identifier for the target building
            unit_id: Identifier for the target unit
            
        Returns:
            bool: True if exploration succeeded, False otherwise
            
        Raises:
            TimeoutError: If exploration server is not available
            Exception: If exploration fails
        """
        self.get_logger().info(f'Starting exploration for building {building_id}, unit {unit_id}')
        try:
            if not await self._wait_for_exploration_server():
                raise TimeoutError('Exploration server not available')

            goal_msg = self._create_exploration_goal(building_id, unit_id)
            goal_future = await self._send_exploration_goal(goal_msg)

            if not goal_future.accepted:
                raise Exception('Exploration goal rejected')

            result = await goal_future.get_result_async()
            return self._handle_exploration_result(result)

        except TimeoutError as e:
            self.get_logger().error(f'Exploration server timeout: {str(e)}')
            raise
        except Exception as e:
            self.get_logger().error(f'Exploration error: {str(e)}')
            raise

    async def _wait_for_exploration_server(self) -> bool:
        """
        Wait for exploration server with timeout
        
        Returns:
            bool: True if server becomes available, False otherwise
        """
        wait_time = 1.0
        max_wait_time = 30.0
        while not self._exploration_client.wait_for_server(timeout_sec=wait_time):
            if wait_time > max_wait_time:
                return False
            self.get_logger().info('Waiting for exploration server...')
            wait_time = min(wait_time * 2, max_wait_time)
        return True

    def _create_exploration_goal(self, building_id: str, unit_id: str) -> EntranceExploration.Goal:
        """
        Create exploration goal message
        
        Args:
            building_id: Target building identifier
            unit_id: Target unit identifier
            
        Returns:
            EntranceExploration.Goal: Configured goal message
        """
        goal_msg = EntranceExploration.Goal()
        goal_msg.building_id = building_id
        goal_msg.unit_id = unit_id
        return goal_msg

    async def _send_exploration_goal(self, goal_msg: EntranceExploration.Goal):
        """
        Send exploration goal to server
        
        Args:
            goal_msg: Goal message to send
            
        Returns:
            Future: Goal response future
        """
        self.get_logger().info('Sending exploration goal...')
        return await self._exploration_client.send_goal_async(
            goal_msg,
            feedback_callback=self._exploration_feedback_callback
        )

    def _handle_exploration_result(self, result) -> bool:
        """
        Handle exploration task result
        
        Args:
            result: Result from exploration task
            
        Returns:
            bool: True if exploration succeeded, False otherwise
        """
        if result.result.success:
            self.get_logger().info(f'Exploration completed successfully: {result.result.message}')
            return True
        else:
            self.get_logger().error(f'Exploration failed: {result.result.message}')
            return False

    def _exploration_feedback_callback(self, feedback_msg):
        """
        Handle feedback from exploration action
        
        Args:
            feedback_msg: Feedback message from exploration server
        """
        self.get_logger().info(f'Exploration status: {feedback_msg.feedback.status}')

    async def _process_delivery_task(self, goal_handle, feedback, feedback_msg, result, result_msg):
        """
        Process and execute delivery task sequence
        
        Args:
            goal_handle: Goal handle for managing task execution
            feedback: Feedback message object
            feedback_msg: Feedback message content
            result: Result message object
            result_msg: Result message content
            
        Returns:
            result: Task execution result
            
        Raises:
            Exception: If task sequence retrieval or execution fails
        """
        try:
            user_input = goal_handle.request.user_input
            self.get_logger().info(f'Starting new delivery task: {user_input}')
            
            task_sequence = await self._get_task_sequence(user_input)
            if task_sequence is None:
                raise Exception("Failed to get task sequence from planning service")
            
            self.get_logger().info(f'Received task sequence with {len(task_sequence)} tasks')
            return await self._execute_task_sequence(
                task_sequence, 
                goal_handle, 
                feedback, 
                feedback_msg, 
                result, 
                result_msg
            )
            
        except Exception as e:
            if goal_handle.is_cancel_requested:
                return self._handle_cancellation(goal_handle, result, result_msg)
            raise Exception(f"Task execution failed: {str(e)}")

    async def _execute_task_sequence(self, task_sequence, goal_handle, feedback, feedback_msg, result, result_msg):
        """
        Execute a sequence of tasks with progress tracking
        
        Args:
            task_sequence: List of tasks to execute
            goal_handle: Goal handle for managing task execution
            feedback: Feedback message object
            feedback_msg: Feedback message content
            result: Result message object
            result_msg: Result message content
            
        Returns:
            result: Task execution result
        """
        total_tasks = len(task_sequence)
        for i, task in enumerate(task_sequence):
            if goal_handle.is_cancel_requested:
                return self._handle_cancellation(goal_handle, result, result_msg)

            self._update_task_progress(
                goal_handle, 
                feedback, 
                feedback_msg,
                f"Starting task {i+1}/{total_tasks}: {task.task_type} - {task.task_information}"
            )
            
            try:
                await self._execute_task(task, goal_handle, feedback, feedback_msg)
            except Exception as e:
                if goal_handle.is_cancel_requested:
                    return self._handle_cancellation(goal_handle, result, result_msg)
                raise
            
            self._update_task_progress(
                goal_handle, 
                feedback, 
                feedback_msg,
                f"Completed task {i+1}/{total_tasks}: {task.task_type} - {task.task_information}"
            )
        
        result_msg.message = "Delivery task completed successfully"
        result.result = result_msg
        goal_handle.succeed()
        return result

    async def _execute_task(self, task, goal_handle, feedback, feedback_msg):
        """
        Execute a single task with appropriate handling based on task type
        
        Args:
            task: Task object containing type and information
            goal_handle: Goal handle for managing task execution
            feedback: Feedback message object
            feedback_msg: Feedback message content
            
        Raises:
            Exception: If task execution fails or task type is unknown
        """
        if task.task_type == "NAVIGATION":
            pass # temporary test
            lon, lat = await self._get_target_coordinates(task.task_information)
            if not await self._navigate_to_pose(lon, lat, goal_handle, feedback, feedback_msg):
                raise Exception(f"Navigation failed: {task.task_information}")
                
        elif task.task_type == "EXPLORATION":
            building_id, unit_id = task.task_information.split(':')
            if not await self._execute_exploration(building_id, unit_id):
                raise Exception(f"Exploration failed: {task.task_information}")
                
        else:
            raise Exception(f"Unknown task type: {task.task_type}")

    def _update_task_progress(self, goal_handle, feedback, feedback_msg, progress_info):
        """
        Update task progress feedback
        
        Args:
            goal_handle: Goal handle for the task
            feedback: Feedback message object
            feedback_msg: Feedback message content
            progress_info: Progress description
        """
        feedback_msg.status = progress_info
        feedback.feedback = feedback_msg
        goal_handle.publish_feedback(feedback)

    def _handle_cancellation(self, goal_handle, result, result_msg):
        """
        Handle task cancellation request
        
        Args:
            goal_handle: Goal handle for the task
            result: Result message object
            result_msg: Result message content
            
        Returns:
            result: Updated result message
        """
        self.get_logger().info('Processing cancellation request')
        result_msg.message = "Task cancelled"
        result.result = result_msg
        goal_handle.canceled()
        self.get_logger().info('Task cancelled')
        return result

    async def execute_callback(self, goal_handle):
        """
        Main callback for handling delivery task execution requests
        
        Args:
            goal_handle: Goal handle containing task information
            
        Returns:
            result: Task execution result
        """
        feedback = DeliveryTask.Feedback()
        feedback_msg = DeliveryFeedback()
        result = DeliveryTask.Result()
        result_msg = DeliveryResult()
        
        try:
            result = await self._process_delivery_task(
                goal_handle, 
                feedback, 
                feedback_msg, 
                result, 
                result_msg
            )
            return result
        except Exception as e:
            self.get_logger().error(f'Error in execution callback: {str(e)}')
            result_msg.message = f"Delivery task failed: {str(e)}"
            result.result = result_msg
            goal_handle.abort()
            return result

    def cancel_callback(self, goal_handle):
        """
        Callback for handling task cancellation requests
        
        Args:
            goal_handle: Goal handle of the task to be cancelled
            
        Returns:
            CancelResponse: Indicates whether cancellation request is accepted
        """
        self.get_logger().info('Received cancellation request')
        
        if hasattr(self, 'navigator'):
            self.navigator.cancelTask()
        
        return CancelResponse.ACCEPT

    def _calculate_waypoint_orientation(self, 
                                     current_idx: int, 
                                     waypoints: list, 
                                     current_utm_east: float, 
                                     current_utm_north: float,
                                     robot_position: tuple) -> tuple:
        """
        Calculate orientation quaternion for waypoint navigation
        
        Args:
            current_idx: Index of current waypoint
            waypoints: List of all waypoints
            current_utm_east: Current waypoint's UTM easting
            current_utm_north: Current waypoint's UTM northing
            robot_position: Robot's current position (utm_east, utm_north)
            
        Returns:
            tuple: Quaternion (x, y, z, w) representing orientation
        """
        try:
            if current_idx == 0:
                # First waypoint: orient towards it from robot position
                dx = current_utm_east - robot_position[0]
                dy = current_utm_north - robot_position[1]
            else:
                # Other waypoints: orient towards next waypoint if available
                next_idx = current_idx + 1
                if next_idx < len(waypoints):
                    next_waypoint = waypoints[next_idx]
                    next_utm_east, next_utm_north, _ = CoordinateTransformer.wgs84_to_utm(
                        next_waypoint.lon, next_waypoint.lat)
                    dx = next_utm_east - current_utm_east
                    dy = next_utm_north - current_utm_north
                else:
                    # Last waypoint: maintain previous orientation
                    return self._yaw_to_quaternion(0.0)

            # Calculate yaw angle and convert to quaternion
            yaw = math.atan2(dy, dx)
            return self._yaw_to_quaternion(yaw)
            
        except Exception as e:
            self.get_logger().error(f'Error calculating orientation: {str(e)}')
            return self._yaw_to_quaternion(0.0)  # Return default orientation on error

def main(args=None):
    """
    Main entry point for the delivery executor node.
    
    Args:
        args: Command line arguments (optional)
    """
    rclpy.init(args=args)
    
    action_server = None
    executor = None
    
    try:
        # Initialize action server
        action_server = DeliveryExecutorActionServer()
        
        # Configure multi-threaded executor
        executor = MultiThreadedExecutor(num_threads=4)  # Adjust thread count as needed
        executor.add_node(action_server)
        
        # Log startup status
        action_server.get_logger().info('Delivery Executor Action Server is ready')
        
        # Start spinning
        try:
            executor.spin()
        except KeyboardInterrupt:
            action_server.get_logger().info('Received keyboard interrupt, shutting down...')
        except Exception as e:
            action_server.get_logger().error(f'Unexpected error during execution: {str(e)}')
            raise
            
    except Exception as e:
        if action_server:
            action_server.get_logger().error(f'Fatal error: {str(e)}')
        else:
            print(f'Failed to initialize action server: {str(e)}')
        raise
        
    finally:
        # Cleanup
        try:
            if action_server:
                if hasattr(action_server, 'navigator'):
                    action_server.navigator.destroy_node()
                action_server.destroy_node()
            if executor:
                executor.shutdown()
            rclpy.shutdown()
        except Exception as e:
            print(f'Error during cleanup: {str(e)}')

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f'Fatal error in main: {str(e)}')
        sys.exit(1)
