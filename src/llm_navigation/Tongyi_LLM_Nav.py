import subprocess
import json
from dashscope import Generation
import osm_route as OsmRoute
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from pyproj import CRS, Transformer

def get_response(messages):
    """Call the Qwen-Turbo model with the provided messages and return the response."""
    response = Generation.call(model="qwen-turbo",
                               messages=messages,
                               result_format='message')
    return response

def read_prompt_file(file_path):
    """Read the initial prompt from a file."""
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read().strip()
    return content

def main():
    """Main function to handle the delivery process."""
    # Read the prompt from a file
    file_path = 'prompt_en.txt'
    initial_prompt = read_prompt_file(file_path)

    # Initialize messages with the system prompt
    messages = [{'role': 'user', 'content': initial_prompt}]
    assistant_output = get_response(messages).output.choices[0]['message']['content']
    messages.append({'role': 'assistant', 'content': assistant_output})

    # Customize the number of conversation rounds, currently set to 1
    for _ in range(1):
        user_input = input("\nHi! I am XiaoZhi~ Do you need any delivery?\n")
        messages.append({'role': 'user', 'content': user_input})
        assistant_output = get_response(messages).output.choices[0]['message']['content']
        messages.append({'role': 'assistant', 'content': assistant_output})

        # Parse the assistant_output as JSON
        assistant_output_dict = json.loads(assistant_output)
        building_positions = assistant_output_dict.get("building_positions", [])

        # Convert building_positions to a list of lists
        building_positions_list = list(map(list, building_positions))

        # Define coordinate reference systems
        wgs84 = CRS.from_epsg(4326)
        utm33n = CRS.from_epsg(32633)
        transformer = Transformer.from_crs(utm33n, wgs84, always_xy=True)

        print(f"\nGreat! There are {len(building_positions_list)} locations for delivery.")
        print(f"Their coordinates are: {building_positions_list}\n")

        curr_robot_position = list(transformer.transform(0.0, 0.0))
        building_positions_list = [curr_robot_position] + building_positions_list

        # Initialize ROS2 and navigator
        rclpy.init()
        navigator = BasicNavigator()

        # Assuming building_positions_list contains multiple positions
        for i in range(len(building_positions_list) - 1):
            print('############################################################')
            print(f"Delivering to location {i + 1}, please wait patiently!\n")

            start_position = f"{building_positions_list[i][0]:.9f},{building_positions_list[i][1]:.9f}"
            end_position = f"{building_positions_list[i + 1][0]:.9f},{building_positions_list[i + 1][1]:.9f}"
            
            waypoints = OsmRoute.get_route(start_position, end_position)

            print(f"There are {len(waypoints)} waypoints to be followed.")

            # If conducting an gazebo experiment, please remove the annotation below.
            ''' 
            if waypoints:
                # Follow waypoints
                navigator.followWaypoints(waypoints)
                while not navigator.isTaskComplete():
                    feedback = navigator.getFeedback()
                    # Uncomment to print feedback
                    # print(feedback)

                print(f"\nDelivery to location {i + 1} completed!\n")
                print(navigator.getResult(), "\n")
            else:
                print(f"Unable to get a valid navigation path from {start_position} to {end_position}\n")
            '''

            print('############################################################\n')

        # Shutdown ROS2
        rclpy.shutdown()

if __name__ == '__main__':
    main()