import cv2
import requests
import base64
import numpy as np
from typing import Dict, List, Optional, Tuple

def process_image(img_data: np.ndarray, text_prompt: str, server_url: str = "http://localhost:5000/process_image") -> Optional[Dict]:
    """
    Send image to server for processing and return the results
    
    Args:
        img_data: OpenCV image data (numpy array)
        text_prompt: Text prompt for object detection
        server_url: URL of the processing server
        
    Returns:
        Dict containing processed results or None if request fails
    """

    _, buffer = cv2.imencode('.jpg', img_data)
    encoded_image = base64.b64encode(buffer).decode('utf-8')
    
    data = {
        "img_base64": encoded_image,
        "text_prompt": text_prompt
    }
    
    response = requests.post(server_url, json=data)
    
    if response.status_code != 200:
        print(f"Request failed with status code: {response.status_code}")
        return None
        
    try:
        return response.json()
    except ValueError:
        print("Failed to parse JSON response")
        return None

def decode_image(encoded_data: str) -> Optional[np.ndarray]:
    """Convert base64 encoded image data to numpy array"""
    try:
        image_data = base64.b64decode(encoded_data)
        nparr = np.frombuffer(image_data, np.uint8)
        return cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    except Exception as e:
        print(f"Failed to decode image: {e}")
        return None

def visualize_results(response_data: Dict) -> None:
    """Visualize the processed image and masks"""  
    if labels := response_data.get("labels"):
        print("Labels:", labels)
    
    if masks := response_data.get("masks"):
        for i, mask in enumerate(masks):
            mask_array = np.array(mask, dtype=np.uint8) * 255
            cv2.imshow(f'Mask {i}', mask_array)
            cv2.waitKey(0)
    
    cv2.destroyAllWindows()

def main():
    img_path = "/workspaces/light-map-navigation/test.jpg"
    text_prompt = "road . car"
    
    img_data = cv2.imread(img_path)
    
    if response_data := process_image(img_data, text_prompt):
        visualize_results(response_data)

if __name__ == "__main__":
    main()