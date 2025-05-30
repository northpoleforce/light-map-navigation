import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
from custom_interfaces.srv import GetEntranceId
import base64
import json
import yaml
import os
import re
import requests
from PIL import Image
from io import BytesIO

class EntranceRecognitionService(Node):
    SYSTEM_PROMPT = """Now I need your help to analysis the picture as my assistant. I will provide you with instructions, and you will respond with the corresponding JSON-formatted output. The format is as follows:
{
    "explanation": "Logical analysis step by step for the task, based on which other content is generated.",
    "unit_number": "Only the unit number." # If you cannot find the unit number, please return an empty string.
}
For example:
{
    "explanation": "The picture shows Unit 2.",
    "unit_number": "2"
}
Note: It is necessary to output the result in JSON format and nothing else."""

    def __init__(self):
        super().__init__('entrance_recognition_server')
        self.srv = self.create_service(GetEntranceId, 'entrance_recognition', self.recognize_entrance_callback)
        self.bridge = CvBridge()
        self._init_api_config()

    def _init_api_config(self):
        """Initialize API configuration from yaml file"""
        try:
            # Get the package path
            package_path = os.path.dirname(os.path.dirname(__file__))
            config_path = os.path.join(package_path, 'config', 'api_config.yaml')
            
            # Load configuration
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
            
            # Store API configuration
            self.api_url = config['vlm_api']['base_url']
            self.get_logger().info('API configuration loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load API configuration: {str(e)}')
            raise

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
        Recognize entrance unit number from image using LLM API
        
        Args:
            image (np.ndarray): OpenCV image in BGR format
            
        Returns:
            str: Unit number if detected, None otherwise
        """
        try:
            # 转换为PIL图像进行处理
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(image_rgb)
            
            # 转换为base64
            image_base64 = self._convert_image_to_base64(pil_image)
            
            # 获取LLM响应
            response = self._get_llm_response(image_base64)
            
            # 解析并验证响应
            return self._parse_llm_response(response)
            
        except Exception as e:
            self.get_logger().error(f"Error in entrance recognition: {str(e)}")
            return None

    def _convert_image_to_base64(self, pil_image, size=(512, 512)):
        """
        将PIL图像转换为base64字符串
        
        Args:
            pil_image (PIL.Image): PIL图像对象
            size (tuple): 目标图像大小 (width, height)，默认 (512, 512)
            
        Returns:
            str: Base64编码的图像字符串
        """
        # 转为RGB格式
        pil_image = pil_image.convert("RGB")
        # 缩放到指定尺寸
        pil_image = pil_image.resize(size)
        
        # 转换为BytesIO以便编码
        buffered = BytesIO()
        pil_image.save(buffered, format="JPEG")
        # base64编码
        img_base64 = base64.b64encode(buffered.getvalue()).decode("utf-8")
        return img_base64

    def _get_llm_response(self, image_base64):
        """
        调用LLM API获取响应
        
        Args:
            image_base64 (str): Base64编码的图像字符串
            
        Returns:
            dict: API响应
        """
        # 构造请求数据
        prompt = f"Please analyze this image and identify the unit number. {self.SYSTEM_PROMPT}"
        
        payload = {
            "image_base64": image_base64,
            "prompt": prompt
        }

        try:
            single_image_api = f"{self.api_url}/inference/single_image"
            self.get_logger().info(f"Calling API at: {single_image_api}")
            
            response = requests.post(single_image_api, json=payload)
            response.raise_for_status()  # 如果非2xx，会抛出异常
            return response.json()
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"API request error: {str(e)}")
            return None

    def _parse_llm_response(self, response):
        """解析并验证LLM响应"""
        if not response:
            return None
            
        try:
            # 新接口返回的格式处理
            if "response" in response:
                response_text = response["response"]
                
                # 使用正则表达式查找JSON对象
                pattern = r'\{[^{}]*\}'
                matches = re.findall(pattern, response_text)
                if matches:
                    json_str = matches[0]
                    result = json.loads(json_str)
                    # 获取单元号
                    return result.get("unit_number")
                else:
                    self.get_logger().error("No JSON object found in response")
                    self.get_logger().error(f"Raw response: {response_text}")
            else:
                self.get_logger().error("Invalid API response format")
                self.get_logger().error(f"Response: {response}")
                
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"Failed to parse LLM response: {e}")
            self.get_logger().error(f"Raw response: {response}")
        return None

def main(args=None):
    rclpy.init(args=args)
    node = EntranceRecognitionService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()