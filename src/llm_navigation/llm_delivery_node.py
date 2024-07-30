import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Pose, PoseStamped
from pyproj import CRS, Transformer
import llm_agent as LLMAgent
import osm_route as OsmRoute
from tf2_ros import TransformListener, Buffer

class DeliveryActionClient(Node):
    def __init__(self):
        super().__init__('delivery_action_client')

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.navigator = BasicNavigator()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.wgs84 = CRS.from_epsg(4326)
        self.utm33n = CRS.from_epsg(32633)
        self.transformer = Transformer.from_crs(self.utm33n, self.wgs84, always_xy=True)
        self.current_waypoint_index = 0
        self.waypoints = []

        self.robot_position = None
        self.position_subscription = self.create_subscription(
            Pose,
            '/robot_position',
            self.position_callback,
            20
        )

    def position_callback(self, msg):
        self.robot_position = [msg.position.x, msg.position.y]
        # self.get_logger().info(f"Received robot position: x={msg.position.x}, y={msg.position.y}")

    def get_robot_position(self):
        return self.robot_position

    def send_goal(self, waypoint):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(f'Received feedback: {feedback}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')

        self.current_waypoint_index += 1
        if self.current_waypoint_index < len(self.waypoints):
            next_waypoint = self.waypoints[self.current_waypoint_index]
            self.send_goal(next_waypoint)
        else:
            self.get_logger().info("All waypoints completed.")
            rclpy.shutdown()

def main(args=None):
    user_input = input("\nHi! I am XiaoZhi~ Do you need any delivery?\n")
    response = LLMAgent.call_llm(user_input)
    building_coords, _ = LLMAgent.extract_coordinates(response)

    for i, position in enumerate(building_coords, start=1):
        rclpy.init(args=args)
        delivery_client = DeliveryActionClient()

        delivery_client.get_logger().info(f"Delivering to location {i}, please wait patiently!\n")

        try:
            while rclpy.ok() and delivery_client.get_robot_position() is None:
                rclpy.spin_once(delivery_client, timeout_sec=1.0)
            robot_position = delivery_client.get_robot_position()

            if robot_position is None:
                delivery_client.get_logger().error("Robot position is not available.")
                continue

            curr_robot_position = list(delivery_client.transformer.transform(robot_position[0], robot_position[1]))

            start_position = f"{curr_robot_position[0]:.9f},{curr_robot_position[1]:.9f}"
            end_position = f"{position[0]:.9f},{position[1]:.9f}"

            waypoints = OsmRoute.get_route(start_position, end_position)
            if waypoints:
                delivery_client.waypoints = waypoints
                delivery_client.send_goal(delivery_client.waypoints[0])
                rclpy.spin(delivery_client)
            else:
                delivery_client.get_logger().info(f"Unable to get a valid navigation path from {start_position} to {end_position}\n")

        except Exception as e:
            delivery_client.get_logger().error(f"Error occurred: {e}")

        finally:
            if rclpy.ok():
                rclpy.shutdown()

        delivery_client.get_logger().info(f"\nDelivered to location {i}, good job!\n")

if __name__ == '__main__':
    main()