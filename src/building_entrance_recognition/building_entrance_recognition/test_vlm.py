import cv2
import base64
import json
import yaml
import os
import re
import requests
from PIL import Image
from io import BytesIO

class EntranceRecognitionService:
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

    def __init__(self, config_path=None):
        self._init_api_config(config_path)

    def _init_api_config(self, config_path=None):
        """Initialize API configuration from yaml file"""
        try:
            if config_path is None:
                package_path = os.path.dirname(os.path.abspath(__file__))
                config_path = os.path.join(package_path, 'config', 'api_config.yaml')
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
            self.api_url = config['vlm_api']['base_url']
            print('API configuration loaded successfully')
        except Exception as e:
            print(f'Failed to load API configuration: {str(e)}')
            raise

    def recognize_entrance(self, image):
        """
        Recognize entrance unit number from image using LLM API

        Args:
            image (np.ndarray): OpenCV image in BGR format

        Returns:
            str: Unit number if detected, None otherwise
        """
        try:
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(image_rgb)
            image_base64 = self._convert_image_to_base64(pil_image)
            response = self._get_llm_response(image_base64)
            return self._parse_llm_response(response)
        except Exception as e:
            print(f"Error in entrance recognition: {str(e)}")
            return None

    def _convert_image_to_base64(self, pil_image, size=(512, 512)):
        pil_image = pil_image.convert("RGB")
        pil_image = pil_image.resize(size)
        buffered = BytesIO()
        pil_image.save(buffered, format="JPEG")
        img_base64 = base64.b64encode(buffered.getvalue()).decode("utf-8")
        return img_base64

    def _get_llm_response(self, image_base64):
        prompt = f"Please analyze this image and identify the unit number. {self.SYSTEM_PROMPT}"
        payload = {
            "image_base64": image_base64,
            "prompt": prompt
        }
        try:
            single_image_api = f"{self.api_url}/inference/single_image"
            print(f"Calling API at: {single_image_api}")
            response = requests.post(single_image_api, json=payload)
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"API request error: {str(e)}")
            return None

    def _parse_llm_response(self, response):
        if not response:
            return None
        try:
            if "response" in response:
                response_text = response["response"]
                pattern = r'\{[^{}]*\}'
                matches = re.findall(pattern, response_text)
                if matches:
                    json_str = matches[0]
                    result = json.loads(json_str)
                    return result.get("unit_number")
                else:
                    print("No JSON object found in response")
                    print(f"Raw response: {response_text}")
            else:
                print("Invalid API response format")
                print(f"Response: {response}")
        except (json.JSONDecodeError, KeyError) as e:
            print(f"Failed to parse LLM response: {e}")
            print(f"Raw response: {response}")
        return None

def main():
    # 示例：读取图片并识别
    image_path = "test.png"  # 替换为你的图片路径
    config_path = "../config/api_config.yaml"  # 替换为你的配置文件路径
    image = cv2.imread(image_path)
    if image is None:
        print(f"Failed to load image: {image_path}")
        return
    recognizer = EntranceRecognitionService(config_path)
    unit_number = recognizer.recognize_entrance(image)
    if unit_number:
        print(f"Detected entrance unit number: {unit_number}")
    else:
        print("No entrance detected.")

if __name__ == '__main__':
    main()