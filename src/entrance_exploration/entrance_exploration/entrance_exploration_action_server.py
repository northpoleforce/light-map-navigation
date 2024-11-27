import rclpy
from rclpy.action import (
    ActionServer,
    GoalResponse,
    CancelResponse
)
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import (
    TransformStamped,
    PoseStamped
)
from sensor_msgs.msg import Image
from nav2_simple_commander.robot_navigator import (
    BasicNavigator,
    TaskResult
)

from custom_interfaces.action import EntranceExploration
from custom_interfaces.srv import GetEntranceId
import entrance_exploration.exploration_route as expRoute

import time

class EntranceExplorationActionServer(Node):
    def __init__(self):
        super().__init__('entrance_exploration_action_server')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('osm_file_path', '/workspaces/light-map-navigation/src/llm_exploration_py/OSM/medium.osm'),
                ('exploration_radius', 2.0),
                ('exploration_points', 5),
                ('camera_topic', '/camera_sensor/image_raw'),
                ('map_frame', 'map'),
                ('odom_frame', 'lidar_odom')
            ]
        )
        
        self.osm_file_path = self.get_parameter('osm_file_path').value
        self.exploration_radius = self.get_parameter('exploration_radius').value
        self.exploration_points = self.get_parameter('exploration_points').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        
        # create reentrant callback group
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize action server with goal and cancel callbacks
        self._action_server = ActionServer(
            self,
            EntranceExploration,
            'explore_entrance',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        
        # Initialize navigation components
        self._init_navigation()
        
        # Initialize communication components
        self._init_perception()
        
        # Initialize state variables
        self._init_state_variables()
            
    def _init_navigation(self):
        """Initialize navigation-related components"""
        self.navigator = BasicNavigator()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
    def _init_perception(self):
        """Initialize perception-related components"""
        # Service client
        self.get_entrance_id_client = self.create_client(GetEntranceId, 'entrance_recognition')
        self._wait_for_service()
        
        # Image subscriber
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10)
            
    def _init_state_variables(self):
        """Initialize state tracking variables"""
        self.latest_image = None
        self.cur_position = None
        self.current_waypoint_index = 0
        self.waypoint_ = []
        self.is_task_success = False
        self._cancel_requested = False
        self._current_goal_handle = None

    def _reset_state(self):
        """Reset all state variables for new goal"""
        self.current_waypoint_index = 0
        self.waypoint_ = []
        self.is_task_success = False
        self._cancel_requested = False
        self._current_goal_handle = None

    def _wait_for_service(self):
        """Wait for required services to become available"""
        while not self.get_entrance_id_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for entrance_recognition service...')

    def image_callback(self, msg):
        """Callback function for receiving image data"""
        self.latest_image = msg
            
    def goal_callback(self, goal_request):
        """
        Callback for handling new goal requests
        Args:
            goal_request: The goal request message
        Returns:
            GoalResponse: Accept or reject the goal request
        """
        self.get_logger().info('Received goal request')
        
        # Validate goal parameters
        try:
            if not isinstance(goal_request.building_id, str) or not isinstance(goal_request.unit_id, str):
                self.get_logger().error('Invalid goal: building_id and unit_id must be strings')
                return GoalResponse.REJECT
                
            if not goal_request.building_id or not goal_request.unit_id:
                self.get_logger().error('Invalid goal: building_id and unit_id cannot be empty')
                return GoalResponse.REJECT
                
            # If there is an active task, reject the new request
            if hasattr(self, '_current_goal_handle') and self._current_goal_handle is not None:
                self.get_logger().warn('Another goal is already active, rejecting new goal')
                return GoalResponse.REJECT
                
            self.get_logger().info(f'Goal accepted - Building: {goal_request.building_id}, Unit: {goal_request.unit_id}')
            return GoalResponse.ACCEPT
            
        except Exception as e:
            self.get_logger().error(f'Error in goal validation: {str(e)}')
            return GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        """
        Callback for handling cancellation requests
        Args:
            goal_handle: The goal handle being cancelled
        Returns:
            CancelResponse: Accept or reject the cancel request
        """
        self.get_logger().info('Received cancel request')
        
        try:
            if not hasattr(self, '_current_goal_handle') or self._current_goal_handle is None:
                self.get_logger().warn('No active goal to cancel')
                return CancelResponse.REJECT
                
            if goal_handle != self._current_goal_handle:
                self.get_logger().warn('Cancel request does not match current goal')
                return CancelResponse.REJECT
                
            self._cancel_requested = True
            
            # cancel navigation task
            if hasattr(self, 'navigator'):
                self.navigator.cancelTask()
                
            self.get_logger().info('Cancel request accepted')
            return CancelResponse.ACCEPT
            
        except Exception as e:
            self.get_logger().error(f'Error in cancel callback: {str(e)}')
            return CancelResponse.REJECT

    async def execute_callback(self, goal_handle):
        """Main execution callback for the action server"""
        self._current_goal_handle = goal_handle
        self._cancel_requested = False

        try:
            self.get_logger().info('Executing goal...')

            # wait for transform availability
            try:
                transform: TransformStamped = await self.tf_buffer.lookup_transform_async(
                    self.map_frame,
                    self.odom_frame,
                    rclpy.time.Time())
            except Exception as e:
                self.get_logger().error(f'Transform lookup failed: {str(e)}')
                goal_handle.abort()
                return EntranceExploration.Result(
                    success=False,
                    message=f'Transform lookup failed: {str(e)}'
                )
            
            self.cur_position = (transform.transform.translation.x, transform.transform.translation.y)
            self.get_logger().info(f'Current position: {self.cur_position}')
            
            # get target parameters
            self.target_building_id = goal_handle.request.building_id
            self.target_unit_id = goal_handle.request.unit_id
            
            # Parameter validation
            if not isinstance(self.target_building_id, str) or not isinstance(self.target_unit_id, str):
                goal_handle.abort()
                return EntranceExploration.Result(
                    success=False,
                    message="building_id and unit_id must be strings"
                )
            
            if not self.target_building_id or not self.target_unit_id:
                goal_handle.abort()
                return EntranceExploration.Result(
                    success=False,
                    message="building_id and unit_id cannot be empty"
                )

            try:
                self.waypoint_ = expRoute.execute_exploration(
                    self.osm_file_path,
                    self.target_building_id, 
                    self.exploration_radius,
                    self.exploration_points, 
                    self.cur_position
                )
                self.get_logger().info(f"Waypoints: {self.waypoint_}")
            except Exception as e:
                self.get_logger().error(f"Failed to get waypoints: {str(e)}")
                goal_handle.abort()
                return EntranceExploration.Result(
                    success=False,
                    message=f"Failed to get waypoints: {str(e)}"
                )
                
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {str(e)}')
            goal_handle.abort()
            return EntranceExploration.Result(
                success=False,
                message=f'Exploration failed due to unexpected error: {str(e)}'
            )

        result = await self.exec_navigation(goal_handle)
        
        if self._cancel_requested:
            self.get_logger().info("Goal was canceled")
            goal_handle.canceled()
        elif result.success:
            self.get_logger().info("Goal completed successfully")
            goal_handle.succeed()
        else:
            self.get_logger().info("Goal failed")
            goal_handle.abort()

        self._reset_state()
        return result

    async def exec_navigation(self, goal_handle):
        """
        Execute navigation sequence through waypoints
        Args:
            goal_handle: The goal handle for the action
        Returns:
            EntranceExploration.Result: The result of the exploration
        """
        feedback_msg = EntranceExploration.Feedback()
        
        try:
            while self.current_waypoint_index < len(self.waypoint_) and not self._cancel_requested:
                feedback_msg.status = f"Exploring waypoint {self.current_waypoint_index + 1}/{len(self.waypoint_)}"
                goal_handle.publish_feedback(feedback_msg)
                
                if await self._navigate_to_waypoint(goal_handle):
                    if await self._check_entrance(goal_handle):
                        self.is_task_success = True
                        return EntranceExploration.Result(
                            success=True,
                            message="Successfully found the target entrance"
                        )
                self.current_waypoint_index += 1

            # Check if the task was canceled
            if self._cancel_requested:
                return EntranceExploration.Result(
                    success=False,
                    message="Task was canceled"
                )

            return EntranceExploration.Result(
                success=self.is_task_success,
                message="Completed exploration without finding target entrance"
            )
            
        except Exception as e:
            self.get_logger().error(f"Exception in exec_navigation: {str(e)}")
            return EntranceExploration.Result(
                success=False,
                message=f"Navigation failed: {str(e)}"
            )

    async def _navigate_to_waypoint(self, goal_handle):
        """
        Navigate to a single waypoint
        Returns:
            bool: True if navigation succeeded, False otherwise
        """
        waypoint = self.waypoint_[self.current_waypoint_index]
        goal = self._create_pose_goal(waypoint)
        
        self.navigator.goToPose(goal)
        self.get_logger().info(f"Sent waypoint {self.current_waypoint_index + 1}: ({waypoint[0]}, {waypoint[1]})")
        
        return await self._wait_for_navigation(goal_handle)

    def _create_pose_goal(self, waypoint):
        """Create a PoseStamped message from waypoint coordinates"""
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = waypoint[0]
        goal.pose.position.y = waypoint[1]
        goal.pose.position.z = 0.0
        goal.pose.orientation = expRoute.euler_to_quaternion(waypoint[2])
        return goal

    async def _wait_for_navigation(self, goal_handle):
        """
        Wait for the navigation task to complete and publish navigation progress
        Returns:
            bool: True if navigation succeeded, False otherwise
        """
        feedback_msg = EntranceExploration.Feedback()
        
        while not self.navigator.isTaskComplete():
            if self._cancel_requested:
                self.get_logger().info('Navigation task was cancelled')
                feedback_msg.status = "Navigation cancelled"
                goal_handle.publish_feedback(feedback_msg)
                return False
                
            # Get navigation feedback
            nav_feedback = self.navigator.getFeedback()
            if nav_feedback:
                feedback_msg.status = (
                    f"Navigating to waypoint {self.current_waypoint_index + 1}/{len(self.waypoint_)}, "
                    f"Distance remaining: {nav_feedback.distance_remaining:.2f}m"
                )
                goal_handle.publish_feedback(feedback_msg)
                
                self.get_logger().info(feedback_msg.status)
            
            time.sleep(2)

        # If the task is canceled, return False
        if self._cancel_requested:
            return False

        self.nav_result = self.navigator.getResult()
        if self.nav_result == TaskResult.SUCCEEDED:
            self.get_logger().info("Arrived at target waypoint")
            feedback_msg.status = "Reached waypoint, checking for entrance"
            goal_handle.publish_feedback(feedback_msg)
            return True
        else:
            self.get_logger().info(f'Navigation result: {self.nav_result}')
            feedback_msg.status = f"Navigation ended: {self.nav_result}"
            goal_handle.publish_feedback(feedback_msg)
            return False

    async def _check_entrance(self, goal_handle):
        """
        Check if the current location matches the target entrance
        Args:
            goal_handle: The goal handle for the action
        Returns:
            bool: True if entrance is found and matches target, False otherwise
        """
        feedback_msg = EntranceExploration.Feedback()
        
        if self.latest_image is None:
            self.get_logger().warn('No image data available')
            feedback_msg.status = "Waiting for camera image..."
            goal_handle.publish_feedback(feedback_msg)
            return False

        try:
            # Update status to show we're processing
            feedback_msg.status = "Processing image for entrance detection..."
            goal_handle.publish_feedback(feedback_msg)

            # Create service request
            request = GetEntranceId.Request()
            request.image = self.latest_image

            # Call entrance recognition service
            future = self.get_entrance_id_client.call_async(request)
            feedback_msg.status = "Calling entrance recognition service..."
            goal_handle.publish_feedback(feedback_msg)
            
            response = await future

            # Check if the detected entrance matches our target
            if response.success:
                detected_msg = f'Detected entrance: Unit {response.entrance_id}'
                self.get_logger().info(detected_msg)
                feedback_msg.status = detected_msg
                goal_handle.publish_feedback(feedback_msg)

                if f"unit{response.entrance_id}" == self.target_unit_id:
                    feedback_msg.status = "Target entrance found!"
                    goal_handle.publish_feedback(feedback_msg)
                    return True
                else:
                    feedback_msg.status = "Entrance detected but not the target one, continuing search..."
                    goal_handle.publish_feedback(feedback_msg)
                    return False
            else:
                self.get_logger().warn('No entrance detected in current image')
                feedback_msg.status = "No entrance detected in current view"
                goal_handle.publish_feedback(feedback_msg)
                return False

        except Exception as e:
            error_msg = f'Error during entrance check: {str(e)}'
            self.get_logger().error(error_msg)
            feedback_msg.status = f"Error in entrance detection: {str(e)}"
            goal_handle.publish_feedback(feedback_msg)
            return False

def main():
    rclpy.init()
    entrance_exploration_action_server = EntranceExplorationActionServer()

    executor = MultiThreadedExecutor()
    
    try:
        rclpy.spin(entrance_exploration_action_server, executor=executor)
    except KeyboardInterrupt:
        pass
    
    entrance_exploration_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()