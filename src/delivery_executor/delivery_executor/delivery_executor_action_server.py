import rclpy
from rclpy.node import Node
from rclpy.action import (
    ActionServer,
    ActionClient,
    CancelResponse
)
from action_msgs.msg import GoalStatus
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
import numpy as np
import asyncio


class DeliveryExecutorActionServer(Node):
    """Action server node for handling delivery tasks."""
    
    def __init__(self):
        super().__init__('delivery_executor_action_server')
        
        # Declare all parameters
        self._declare_parameters()
        
        try:
            self._init_components()
            self._init_clients()
            self.get_logger().info('Delivery Executor Action Server started successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize action server: {str(e)}')
            raise

    def _declare_parameters(self):
        """Declare all required ROS parameters"""
        params = {
            'osm_file_path': 'medium.osm',
            'osm_routing_url': 'http://101.200.33.217:30457/route/v1/driving/',
            'transform_matrix': [1.0, 0.0, 500000.0, 0.0, 1.0, 4483000.0, 0.0, 0.0, 1.0],
            'service_wait_time': 1.0,
            'max_service_wait_time': 10.0,
            'exploration_wait_time': 1.0,
            'max_exploration_wait_time': 30.0,
            'navigation_check_interval': 2.0,
            'executor_threads': 4
        }
        
        for name, default in params.items():
            self.declare_parameter(name, default)

    def _init_components(self):
        callback_group = ReentrantCallbackGroup()
        
        self._action_server = ActionServer(
            self,
            DeliveryTask,
            'execute_delivery',
            self.execute_callback,
            callback_group=callback_group,
            cancel_callback=self.cancel_callback
        )
        
        self._init_map_handler()
        self._init_tf_listener()

    def _init_map_handler(self):
        try:
            osm_file = self.get_parameter('osm_file_path').value
            
            self.osm_handler = OSMHandler()
            self.osm_handler.apply_file(osm_file)
        except Exception as e:
            self.get_logger().error(f'Failed to initialize map handler: {str(e)}')
            raise

    def _init_tf_listener(self):
        try:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        except Exception as e:
            self.get_logger().error(f'Failed to initialize TF listener: {str(e)}')
            raise

    def _init_clients(self):
        try:
            self._init_task_planning_client()
            self._init_navigation_client()
            self._init_exploration_client()
        except Exception as e:
            self.get_logger().error(f'Failed to initialize clients: {str(e)}')
            raise

    async def _get_target_coordinates(self, task_info: str) -> tuple:
        info = task_info.lower().replace(" ", "")
        
        try:
            if ':' in info:
                return await self._get_unit_coordinates(info)
            return await self._get_building_coordinates(info)
            
        except Exception as e:
            self.get_logger().error(f'Failed to get coordinates: {str(e)}')
            raise

    async def _get_unit_coordinates(self, info: str) -> tuple:
        building_id, unit_id = info.split(':')
        
        unit_locations = self.osm_handler.get_node_location_by_name(unit_id)
        if not unit_locations:
            raise Exception(f"Unit not found: {unit_id}")
            
        return unit_locations[0]

    async def _get_building_coordinates(self, info: str) -> tuple:
        building_centers = self.osm_handler.get_way_center_by_name(info)
        if not building_centers:
            raise Exception(f"Building not found: {info}")
            
        return building_centers[0]

    def _init_task_planning_client(self):
        self.task_planning_client = self.create_client(
            GetTaskSequence,
            'task_planning'
        )
        
    async def _wait_for_task_planning_service(self):
        wait_time = 1.0
        max_wait_time = 10.0
        while not self.task_planning_client.wait_for_service(timeout_sec=wait_time):
            if wait_time > max_wait_time:
                raise TimeoutError("Task planning service not available after maximum wait time")
            self.get_logger().info('Task planning service not available, waiting...')
            wait_time = min(wait_time * 2, max_wait_time)

    async def _get_task_sequence(self, task_description: str):
        try:
            await self._wait_for_task_planning_service()
            
            request = GetTaskSequence.Request()
            request.task_description = task_description
            
            future = await self.task_planning_client.call_async(request)
            
            if future is not None:
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
        try:
            self.navigator = BasicNavigator()
            self._configure_navigation_parameters()
        except Exception as e:
            self.get_logger().error(f'Failed to initialize navigation client: {str(e)}')
            raise

    def _configure_navigation_parameters(self):
        try:
            # TODO: Add specific navigation parameter configurations
            pass
        except Exception as e:
            self.get_logger().error(f'Failed to configure navigation parameters: {str(e)}')
            raise

    async def _get_robot_position(self) -> tuple:
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
        try:
            robot_position = await self._get_robot_position()
            if not self._validate_robot_position(robot_position):
                return False

            waypoints = await self._get_navigation_waypoints(robot_position, lon, lat)
            return await self._execute_waypoint_navigation(waypoints, robot_position, goal_handle, feedback, feedback_msg) if waypoints else False

        except Exception as e:
            self.get_logger().error(f'Navigation failed: {str(e)}')
            return False

    def _validate_robot_position(self, position: tuple) -> bool:
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

    def _get_transform_matrix(self):
        """Get the transformation matrix from flat array parameter"""
        flat_matrix = self.get_parameter('transform_matrix').value
        return np.array(flat_matrix).reshape(3, 3)

    async def _get_navigation_waypoints(self, robot_position: tuple, target_lon: float, target_lat: float):
        try:
            utm_epsg = CoordinateTransformer.get_utm_epsg(target_lon, target_lat)
            self.get_logger().debug(f'UTM EPSG: {utm_epsg}')
            
            transform_matrix = self._get_transform_matrix()
            
            position_vector = np.array([[robot_position[0]], [robot_position[1]], [1]])
            transformed_position = np.dot(transform_matrix, position_vector)
            
            utm_x = float(transformed_position[0][0])
            utm_y = float(transformed_position[1][0])
            
            curr_robot_lon, curr_robot_lat = CoordinateTransformer.utm_to_wgs84(
                utm_x, utm_y, utm_epsg)
            
            start_pos = f"{float(curr_robot_lon):.9f},{float(curr_robot_lat):.9f}"
            target_pos = f"{float(target_lon):.9f},{float(target_lat):.9f}"
            
            self.get_logger().debug(f'Start position: {start_pos}')
            self.get_logger().debug(f'Target position: {target_pos}')
            
            routing_url = self.get_parameter('osm_routing_url').value
            planner = OsmGlobalPlanner(routing_url)
            waypoints = planner.get_route(start_pos, target_pos)
            self.get_logger().debug(f'Waypoints: {waypoints}')
            
            if not waypoints:
                self.get_logger().error('Failed to get navigation waypoints')
                return None
                
            return waypoints
            
        except Exception as e:
            self.get_logger().error(f'Error getting waypoints: {str(e)}')
            return None

    async def _execute_waypoint_navigation(self, waypoints, robot_position: tuple, goal_handle, feedback, feedback_msg) -> bool:
        total_waypoints = len(waypoints)
        for i, waypoint in enumerate(waypoints):
            if goal_handle.is_cancel_requested:
                return False

            self._update_task_progress(
                goal_handle,
                feedback,
                feedback_msg,
                f"Navigating to waypoint {i+1}/{total_waypoints}"
            )
            
            local_coords = self._transform_coordinates(waypoint)
            pose = self._create_navigation_pose(
                local_coords[0],
                local_coords[1],
                self._calculate_waypoint_orientation(i, waypoints, *local_coords, robot_position)
            )
            
            if not await self._navigate_to_waypoint(pose, i+1, total_waypoints, goal_handle, feedback, feedback_msg):
                return False
                
        return True

    def _transform_coordinates(self, waypoint):
        utm_east, utm_north, _ = CoordinateTransformer.wgs84_to_utm(waypoint.lon, waypoint.lat)
        transform_matrix = self._get_transform_matrix()
        transform_matrix_inv = np.linalg.inv(transform_matrix)
        utm_coords = np.array([[utm_east], [utm_north], [1.0]])
        local_coords = np.dot(transform_matrix_inv, utm_coords)
        return float(local_coords[0][0]), float(local_coords[1][0])

    def _create_navigation_pose(self, x: float, y: float, quaternion: tuple) -> PoseStamped:
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
        self.navigator.goToPose(pose)
        
        while not self.navigator.isTaskComplete():
            if goal_handle.is_cancel_requested:
                self.navigator.cancelTask()
                return False
            
            nav_feedback = self.navigator.getFeedback()
            if nav_feedback:
                status_msg = (
                    f"Task Status: Navigation in Progress\n"
                    f"Progress: Waypoint {current_waypoint}/{total_waypoints}\n"
                    f"Details: Distance remaining - {nav_feedback.distance_remaining:.2f}m"
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
            error_msg = (
                f"Task Status: Navigation Failed\n"
                f"Progress: Waypoint {current_waypoint}/{total_waypoints}\n"
                f"Details: {result}"
            )
            self._update_task_progress(goal_handle, feedback, feedback_msg, error_msg)
            return False
            
        success_msg = (
            f"Task Status: Navigation Succeeded\n"
            f"Progress: Waypoint {current_waypoint}/{total_waypoints}\n"
            f"Details: Reached target position"
        )
        self._update_task_progress(goal_handle, feedback, feedback_msg, success_msg)
        return True

    def _yaw_to_quaternion(self, yaw: float) -> tuple:
        return (
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0)
        )

    def _init_exploration_client(self):
        self._exploration_client = ActionClient(
            self,
            EntranceExploration,
            'explore_entrance'
        )

    async def _execute_exploration(self, building_id: str, unit_id: str, goal_handle, feedback, feedback_msg) -> bool:
        try:
            self._active_goal_handle = goal_handle
            self._active_feedback = feedback
            self._active_feedback_msg = feedback_msg
            self._exploration_goal_handle = None

            init_msg = (
                f"Task Status: Exploration Initializing\n"
                f"Progress: Server Connection\n"
                f"Details: Waiting for exploration server..."
            )
            self._update_task_progress(goal_handle, feedback, feedback_msg, init_msg)

            if not await self._wait_for_exploration_server():
                raise TimeoutError('Exploration server not available')

            goal_msg = self._create_exploration_goal(building_id, unit_id)
            
            start_msg = (
                f"Task Status: Exploration Starting\n"
                f"Progress: Task Setup\n"
                f"Details: Building {building_id}, Unit {unit_id}"
            )
            self._update_task_progress(goal_handle, feedback, feedback_msg, start_msg)
            
            self._exploration_goal_handle = await self._send_exploration_goal(goal_msg)

            if not self._exploration_goal_handle.accepted:
                raise Exception('Exploration goal rejected')

            goal_result_future = self._exploration_goal_handle.get_result_async()

            while not goal_result_future.done():
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Exploration cancelled by client')
                    self._exploration_goal_handle.cancel_goal()
                    return False
                    
                time.sleep(2)

            result = await goal_result_future
            success = self._handle_exploration_result(result)
            
            status_msg = (
                f"Task Status: Exploration {'Completed' if success else 'Failed'}\n"
                f"Progress: Task Finished\n"
                f"Details: {'Successfully explored target area' if success else 'Failed to complete exploration'}"
            )
            self._update_task_progress(goal_handle, feedback, feedback_msg, status_msg)
            
            return success

        except Exception as e:
            self.get_logger().error(f'Exploration error: {str(e)}')
            raise
        finally:
            self._active_goal_handle = None
            self._active_feedback = None
            self._active_feedback_msg = None
            self._exploration_goal_handle = None

    async def _wait_for_exploration_server(self) -> bool:
        wait_time = 1.0
        max_wait_time = 30.0
        while not self._exploration_client.wait_for_server(timeout_sec=wait_time):
            if wait_time > max_wait_time:
                return False
            self.get_logger().info('Waiting for exploration server...')
            wait_time = min(wait_time * 2, max_wait_time)
        return True

    def _create_exploration_goal(self, building_id: str, unit_id: str) -> EntranceExploration.Goal:
        goal_msg = EntranceExploration.Goal()
        goal_msg.building_id = building_id
        goal_msg.unit_id = unit_id
        return goal_msg

    async def _send_exploration_goal(self, goal_msg: EntranceExploration.Goal):
        return await self._exploration_client.send_goal_async(
            goal_msg,
            feedback_callback=self._exploration_feedback_callback
        )

    def _handle_exploration_result(self, result) -> bool:
        if result.result.success:
            self.get_logger().info(f'Exploration completed successfully: {result.result.message}')
            return True
        else:
            self.get_logger().error(f'Exploration failed: {result.result.message}')
            return False

    def _exploration_feedback_callback(self, feedback_msg):
        status = feedback_msg.feedback.status
        
        if hasattr(self, '_active_goal_handle') and hasattr(self, '_active_feedback'):
            exploration_status = (
                f"Task Status: Exploration in Progress\n"
                f"Progress: Entrance Detection\n"
                f"Details: {status}"
            )
            
            self._update_task_progress(
                self._active_goal_handle,
                self._active_feedback,
                self._active_feedback_msg,
                exploration_status
            )

    async def _process_delivery_task(self, goal_handle, feedback, feedback_msg, result, result_msg):
        try:
            user_input = goal_handle.request.user_input
            
            task_sequence = await self._get_task_sequence(user_input)
            if task_sequence is None:
                raise Exception("Failed to get task sequence from planning service")
            
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
        if task.task_type == "NAVIGATION":
            lon, lat = await self._get_target_coordinates(task.task_information)
            if not await self._navigate_to_pose(lon, lat, goal_handle, feedback, feedback_msg):
                raise Exception(f"Navigation failed: {task.task_information}")
                
        elif task.task_type == "EXPLORATION":
            building_id, unit_id = task.task_information.split(':')
            if not await self._execute_exploration(building_id, unit_id, goal_handle, feedback, feedback_msg):
                raise Exception(f"Exploration failed: {task.task_information}")
                
        else:
            raise Exception(f"Unknown task type: {task.task_type}")

    def _update_task_progress(self, goal_handle, feedback, feedback_msg, progress_info):
        feedback_msg.status = progress_info
        feedback.feedback = feedback_msg
        goal_handle.publish_feedback(feedback)

    def _handle_cancellation(self, goal_handle, result, result_msg):
        result_msg.message = "Task cancelled"
        result.result = result_msg
        goal_handle.canceled()
        return result

    async def execute_callback(self, goal_handle):
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
        if hasattr(self, 'navigator'):
            self.navigator.cancelTask()
            
        if hasattr(self, '_exploration_goal_handle') and self._exploration_goal_handle:
            self.get_logger().info('Cancelling exploration task...')
            self._exploration_goal_handle.cancel_goal()
        
        return CancelResponse.ACCEPT

    def _calculate_waypoint_orientation(self, current_idx: int, waypoints: list, 
                                     current_utm_east: float, current_utm_north: float,
                                     robot_position: tuple) -> tuple:
        try:
            transform_matrix = self._get_transform_matrix()
            transform_matrix_inv = np.linalg.inv(transform_matrix)

            if current_idx == 0:
                current_utm = np.array([[current_utm_east], [current_utm_north], [1.0]])
                current_local = np.dot(transform_matrix_inv, current_utm)
                
                dx = current_local[0][0] - robot_position[0]
                dy = current_local[1][0] - robot_position[1]
            else:
                next_idx = current_idx + 1
                if next_idx < len(waypoints):
                    next_waypoint = waypoints[next_idx]
                    next_utm_east, next_utm_north, _ = CoordinateTransformer.wgs84_to_utm(
                        next_waypoint.lon, next_waypoint.lat)
                    
                    current_utm = np.array([[current_utm_east], [current_utm_north], [1.0]])
                    next_utm = np.array([[next_utm_east], [next_utm_north], [1.0]])
                    
                    current_local = np.dot(transform_matrix_inv, current_utm)
                    next_local = np.dot(transform_matrix_inv, next_utm)
                    
                    dx = next_local[0][0] - current_local[0][0]
                    dy = next_local[1][0] - current_local[1][0]
                else:
                    return self._yaw_to_quaternion(0.0)

            yaw = math.atan2(dy, dx)
            return self._yaw_to_quaternion(yaw)
            
        except Exception as e:
            self.get_logger().error(f'Error calculating orientation: {str(e)}')
            return self._yaw_to_quaternion(0.0)

def main(args=None):
    rclpy.init(args=args)
    
    action_server = None
    executor = None
    
    try:
        action_server = DeliveryExecutorActionServer()
        num_threads = action_server.get_parameter('executor_threads').value
        executor = MultiThreadedExecutor(num_threads=num_threads)
        executor.add_node(action_server)
        
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