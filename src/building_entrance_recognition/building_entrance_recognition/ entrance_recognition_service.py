import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
from custom_interfaces.srv import GetEntranceId
import base64
import json
from utils_pkg import APIClient

class EntranceRecognitionService(Node):
    SYSTEM_PROMPT = """Now I need your help to analysis the picture as my assistant. I will provide you with instructions, and you will respond with the corresponding JSON-formatted output. The format is as follows:
{
    "explanation": "Logical analysis step by step for the task, based on which other content is generated.",
    "unit_number": "Only the unit number."
}
For example:
{
    "explanation": "The picture shows Unit 2.",
    "unit_number": "2"
}
Note: It is necessary to output the result in JSON format and nothing else."""

    def __init__(self):
        super().__init__('entrance_recognition_service')
        self.srv = self.create_service(GetEntranceId, 'recognize_entrance', self.recognize_entrance_callback)
        self.bridge = CvBridge()
        self.client = APIClient(
            api_key="sk-729ef159c2b74926874860f6e7e12ca6",
            base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
            model_name="qwen-vl-max"
        )

    def recognize_entrance_callback(self, request, response):
        self.get_logger().info('Received entrance recognition request')

        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(request.image, "bgr8")
            
            # Process the image and get entrance ID
            entrance_id = self.recognize_entrance(cv_image)
            
            if entrance_id:
                response.entrance_id = entrance_id
                response.success = True
                response.message = "Successfully detected entrance"
            else:
                response.entrance_id = ""
                response.success = False
                response.message = "No entrance detected"

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
            response.entrance_id = ""
            response.success = False
            response.message = f"Error: {str(e)}"

        return response

    def recognize_entrance(self, image):
        """
        Implement entrance recognition logic using LLM API
        Args:
            image: OpenCV image
        Returns:
            str: entrance identifier if detected, None otherwise
        """
        try:
            # Convert OpenCV image to base64
            _, buffer = cv2.imencode('.jpg', image)
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            
            # Get response from LLM
            response = self.client.chat(
                prompt="Please analyze this image and identify the unit number.",
                system_prompt=self.SYSTEM_PROMPT,
                image_data=image_base64
            )
            
            if response and response[0]:
                try:
                    result = json.loads(response[0])
                    if result.get("unit_number"):
                        return result["unit_number"]
                except json.JSONDecodeError:
                    self.get_logger().error("Failed to parse LLM response as JSON")
                    
            return None
            
        except Exception as e:
            self.get_logger().error(f"Error in entrance recognition: {str(e)}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = EntranceRecognitionService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
