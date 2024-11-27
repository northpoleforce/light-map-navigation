import rclpy
from rclpy.node import Node
from custom_interfaces.srv import GetTaskSequence
from custom_interfaces.msg import TaskDescription
from utils_pkg import APIClient
import json
import re

class TaskPlanningService(Node):
    """
    ROS2 service node for task planning using LLM.
    Handles task description requests and returns sequence of tasks.
    """
    
    def __init__(self):
        super().__init__('task_planning_service')
        
        # Initialize coordinate data
        self._init_coordinate_data()
        
        # Initialize LLM client
        self._init_llm_client()
        
        # Create service
        self.srv = self.create_service(
            GetTaskSequence,
            'task_planning',
            self.service_callback
        )
        
        self.get_logger().info('Task Planning Service started')

    def _init_coordinate_data(self):
        """Initialize building and unit coordinate data"""
        self.building_coordinates = {
            "building1": (10.511010327, 0.000235218),
            "building2": (10.511016244, 0.000086922),
            "building3": (10.510963920, -0.000079640),
            "building4": (10.510953840, -0.000173880),
            "building5": (10.511022280, -0.000178800),
            "building6": (10.510985680, -0.000295100),
            "building7": (10.511262554, 0.000258062),
            "building8": (10.511312240, 0.000089240),
            "building9": (10.511525424, 0.000177076),
            "building10": (10.511256380, -0.000125360),
            "building11": (10.511509522, -0.000094233),
            "building12": (10.511246718, -0.000243509),
            "building13": (10.511806045, 0.000213527),
            "building14": (10.512026964, 0.000253336),
            "building15": (10.511943636, 0.000122945),
            "building16": (10.512097940, 0.000131020),
            "building17": (10.512033585, -0.000183692),
        }
        
        self.units_coordinates = {
            "building1": {
                "unit1": (),
                "unit2": (),
                "unit3": (),
                "unit4": (),
                "unit5": (),
            },
            "building2": {
                "unit1": (),
                "unit2": (),
            },
            "building3": {
                "unit1": (),
                "unit2": (),
            },
        }

    def _init_llm_client(self):
        """Initialize LLM API client"""
        self.api_client = APIClient(
            api_key="sk-729ef159c2b74926874860f6e7e12ca6",
            base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
            model_name="qwen-turbo"
        )

    def _generate_llm_prompt(self, user_input: str) -> str:
        """
        Generate prompt for LLM
        
        Args:
            user_input: User's task description
            
        Returns:
            Formatted prompt string for LLM
        """
        return f"""
User input: {user_input}

Task: Convert user delivery instructions into structured data.

Key Constraints:
1. Delivery order can be optimized for efficiency
2. Only specified destinations are allowed - no new locations

Reference Data:
- Building coordinates: {self.building_coordinates}
- Unit coordinates: {self.units_coordinates}

Required Output Format (JSON):
{{
    "thinking_process": {{
        "translated_input": "",           # English translation of user input
        "parsed_destinations": {{          # Extracted delivery targets
            "delivery1": "building1 unit2",
            "delivery2": "building1 unit5",
            "delivery3": "building7 unit1"
        }},
        "reasoning": ""                   # Analysis process
    }},
    "delivery_plan": [
        {{
            "item": "",                   # Item to deliver
            "building": {{
                "id": "building1",
                "coordinates_status": "known",    # "known" or "unknown"
                "coordinates": [0.0, 0.0]         # "Empty" if "unknown"
            }},
            "unit": {{
                "id": "unit2",                      # "Empty" or "unit2"
                "coordinates_status": "unknown",    # "known" or "unknown"
                "coordinates": []                   # "Empty" if "unknown"
            }}
        }}
    ]
}}

Output Rules:
1. Each unit should only appear once in the delivery plan
2. Use "coordinates_status": "unknown" when unit coordinates are not available
3. Leave id empty if the unit is not specified in the delivery plan
4. Leave coordinates empty ([]) when unknown
"""

    def call_llm(self, user_input: str) -> str:
        """
        Call LLM with user input and validate response
        
        Args:
            user_input: User's task description
            
        Returns:
            Validated JSON response from LLM
            
        Raises:
            Exception: If LLM response is invalid
        """
        self.get_logger().info(f'User input: {user_input}')
        
        # Get LLM response
        response_content = self.api_client.simple_chat(
            prompt=self._generate_llm_prompt(user_input),
            temperature=0.3
        )
        
        # Validate response
        if response_content is None:
            raise Exception("Empty response from LLM")
            
        self.get_logger().info(f'Raw LLM response: {response_content}')
        
        # Clean up the response using regex pattern matching
        # First remove markdown code block markers if present
        code_block_pattern = r'```(?:json)?\s*([\s\S]*?)```'
        matches = re.findall(code_block_pattern, response_content)
        
        if matches:
            # Use the first match if found
            cleaned_response = matches[0].strip()
        else:
            # If no code block markers found, use the original response
            cleaned_response = response_content.strip()
        
        self.get_logger().info(f'Cleaned response: {cleaned_response}')
        
        try:
            # Validate JSON format
            json.loads(cleaned_response)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON parsing error: {str(e)}')
            self.get_logger().error(f'Cleaned response that failed parsing: {cleaned_response}')
            raise Exception("Invalid JSON format in LLM response")
            
        return cleaned_response

    def extract_coordinates(self, input_json: str) -> tuple:
        """
        Parse JSON and extract coordinate information
        
        Args:
            input_json: JSON string from LLM
            
        Returns:
            Tuple of (building_ids, unit_ids, building_coordinates, unit_coordinates)
            
        Raises:
            Exception: If JSON format is invalid
        """
        try:
            data = json.loads(input_json)
            
            if "delivery_plan" not in data:
                raise Exception("Missing delivery_plan field in JSON")
            
            all_building_ids = []
            all_unit_ids = []
            all_building_coordinates = []
            all_unit_coordinates = []
            
            for delivery in data["delivery_plan"]:
                # Validate required fields
                if "building" not in delivery or "unit" not in delivery:
                    raise Exception("Missing building or unit field in delivery")
                
                building = delivery["building"]
                unit = delivery["unit"]
                
                # Extract building information
                building_id = building.get("id", "")
                building_coords = building.get("coordinates", [])
                
                # Extract unit information
                unit_id = unit.get("id", "")
                unit_coords = unit.get("coordinates", [])
                
                # Append to result lists
                all_building_ids.append(building_id)
                all_unit_ids.append(unit_id)
                all_building_coordinates.append(building_coords)
                all_unit_coordinates.append(unit_coords)
                
            return all_building_ids, all_unit_ids, all_building_coordinates, all_unit_coordinates
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON parsing error: {str(e)}')
            self.get_logger().error(f'Raw JSON: {input_json}')
            raise
        except Exception as e:
            self.get_logger().error(f'Data extraction error: {str(e)}')
            raise

    def service_callback(self, request, response):
        """Handle service requests for task planning"""
        self.get_logger().info('Received service request')
        self.get_logger().info(f'Task description: {request.task_description}')
        
        try:
            # Get planning result from LLM
            llm_response = self.call_llm(request.task_description)
            
            # Extract coordinate information
            building_ids, unit_ids, building_coords, unit_coords = self.extract_coordinates(llm_response)

            self.get_logger().info(f'Building IDs: {building_ids}')
            self.get_logger().info(f'Unit IDs: {unit_ids}')
            self.get_logger().info(f'Building Coordinates: {building_coords}')
            self.get_logger().info(f'Unit Coordinates: {unit_coords}')
            
            # Create task sequence
            task_sequence = []
            for i in range(len(building_ids)):
                task = TaskDescription()
                task.task_type = "NAVIGATION"
                
                if unit_coords[i]: # If unit coordinates are available
                    task.task_information = f"{building_ids[i]}:{unit_ids[i]}"
                else: # If unit coordinates are not available, navigate to the building first, then explore
                    task.task_information = building_ids[i]
                    task_sequence.append(task)
                    
                    # Create exploration task
                    task = TaskDescription()
                    task.task_type = "EXPLORATION" 
                    task.task_information = f"{building_ids[i]}:{unit_ids[i]}"
                
                task_sequence.append(task)
            
            response.task_sequence = task_sequence
            
        except Exception as e:
            self.get_logger().error(f'Service execution error: {str(e)}')
            response.task_sequence = []
            
        return response

def main(args=None):
    """Main function to initialize and run the node"""
    rclpy.init(args=args)
    task_planning_service = TaskPlanningService()
    try:
        rclpy.spin(task_planning_service)
    except KeyboardInterrupt:
        pass
    finally:
        task_planning_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
