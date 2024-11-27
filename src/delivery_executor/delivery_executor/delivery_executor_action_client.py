import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from custom_interfaces.action import DeliveryTask
from rclpy.callback_groups import ReentrantCallbackGroup
import signal
import sys

class DeliveryExecutorActionClient(Node):
    """
    Action client node for initiating and monitoring delivery tasks.
    Provides functionality to send delivery requests and handle responses.
    """

    def __init__(self):
        super().__init__('delivery_executor_action_client')
        self._is_finished = False
        self._goal_handle = None
        
        # Use ReentrantCallbackGroup to allow concurrent execution
        callback_group = ReentrantCallbackGroup()
        
        # Initialize action client
        self._action_client = ActionClient(
            self,
            DeliveryTask,
            'execute_delivery',
            callback_group=callback_group
        )
        
        # Set up signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        self.get_logger().info('Delivery Executor Action Client started')
        self._send_goal_future = None
        self._get_result_future = None

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        self.get_logger().info('Received shutdown signal, canceling task...')
        # Ensure exit even in timeout cases
        try:
            if self._goal_handle is not None:
                cancel_future = self._goal_handle.cancel_goal_async()
                # Reduce timeout duration and add forced exit after timeout
                if not rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=1.0):
                    self.get_logger().warn('Cancel goal timed out, forcing exit...')
            
            self.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {str(e)}')
        finally:
            # Ensure program always exits
            sys.exit(1)

    def cancel_goal(self):
        """Cancel current task with timeout mechanism"""
        if self._goal_handle is not None:
            self.get_logger().info('Canceling current task...')
            try:
                cancel_future = self._goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
                self.get_logger().info('Task canceled')
            except Exception as e:
                self.get_logger().error(f'Error canceling task: {str(e)}')
        else:
            self.get_logger().warn('No active task to cancel')

    def send_goal(self, user_input):
        """
        Send a delivery task goal to the action server.
        
        Args:
            user_input (str): The delivery task details provided by the user
            
        Returns:
            bool: False if input validation fails
        """
        self.get_logger().info('Waiting for Action Server...')
        
        # Wait for server with timeout retry mechanism
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Action Server not available, waiting...')
        
        # Validate user input
        if not user_input or not isinstance(user_input, str):
            self.get_logger().error('Invalid user input')
            return False
        
        # Create and send goal message
        goal_msg = DeliveryTask.Goal()
        goal_msg.user_input = user_input
        
        self.get_logger().info(f'Sending delivery task goal: {user_input}')
        
        # Send goal with feedback callback
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Handle the response to our goal request.
        
        Args:
            future: Future object containing the goal response
        """
        try:
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                self.get_logger().info('Task rejected')
                self._is_finished = True
                return
            
            self.get_logger().info('Task accepted')
            self._goal_handle = goal_handle  # Store goal_handle for cancellation
            
            # Request the result
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)
            
        except Exception as e:
            self.get_logger().error(f'Error handling task response: {str(e)}')
            self._is_finished = True

    def get_result_callback(self, future):
        """
        Process the final result of the delivery task.
        
        Args:
            future: Future object containing the task result
        """
        status = future.result().status
        result = future.result().result
        result_message = result.result.message
        
        # Handle different status outcomes
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Task completed successfully: {result_message}')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'Task canceled: {result_message}')
        else:
            self.get_logger().info(f'Task failed: {result_message}')
        
        self.get_logger().info('Task processing complete, preparing to exit...')
        self._goal_handle = None  # Clear goal handle
        self._is_finished = True
        
        # Shutdown the node
        self.destroy_node()
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """
        Handle feedback updates from the action server.
        
        Args:
            feedback_msg: Contains the DeliveryFeedback message with progress information
        """
        status = feedback_msg.feedback.feedback.status
        self.get_logger().info(f'Received feedback: {status}')

    def destroy_node(self):
        """
        Clean up resources before node shutdown.
        """
        if self._action_client:
            self._action_client.destroy()
        super().destroy_node()

def main():
    """Main function to run the action client."""
    try:
        # Initialize ROS client
        rclpy.init()
        
        # Create and run action client
        action_client = DeliveryExecutorActionClient()
        action_client.send_goal("Please deliver this apple to unit 2 of building 1")
        
        # Spin until shutdown
        rclpy.spin(action_client)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error occurred: {str(e)}")
    finally:
        # Cleanup
        if rclpy.ok():
            try:
                # Ensure the goal is canceled before exiting
                if hasattr(action_client, '_goal_handle') and action_client._goal_handle is not None:
                    action_client.cancel_goal()
                action_client.destroy_node()
            except Exception as e:
                print(f"Error during cleanup: {str(e)}")
            finally:
                rclpy.shutdown()

if __name__ == '__main__':
    main() 