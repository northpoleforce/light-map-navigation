import random
import argparse

def generate_delivery_instructions(num_instructions, instructions_per_block):
    # Define the relationship between buildings and units
    building_units = {
        "building1": ["unit1", "unit2"],
        "building2": ["unit1", "unit2"],
        "building3": ["unit1", "unit2"],
        "building4": ["unit1", "unit2"],
        "building5": ["unit1", "unit2"],
        "building6": ["unit1", "unit2"],
        "building7": ["unit1", "unit2"],
        "building8": ["unit1", "unit2"],
        "building9": ["unit1", "unit2"],
        "building10": ["unit1", "unit2"],
        "building11": ["unit1", "unit2"],
        "building12": ["unit1", "unit2"],
        "building13": ["unit1", "unit2"],
        "building14": ["unit1", "unit2"],
        "building15": ["unit1", "unit2"],
        "building16": ["unit1", "unit2"],
        "building17": ["unit1", "unit2"]
    }

    # Define a list of items that can be delivered
    boxes = ["box", "package", "parcel", "item"]

    instructions = []
    
    for _ in range(num_instructions):
        instruction = ""
        for _ in range(instructions_per_block):
            # Randomly select a box and a building
            selected_box = random.choice(boxes)
            selected_building = random.choice(list(building_units.keys()))

            # Randomly select a unit corresponding to the selected building
            selected_unit = random.choice(building_units[selected_building])

            # Generate the delivery instruction text
            description = f"Please deliver this {selected_box} into {selected_unit}, {selected_building}."
            instruction += " "
            instruction += description
        instructions.append(instruction)

    return instructions

def main(num_instructions, instructions_per_block, output_file):
    # Generate the specified number of delivery instructions and organize them into blocks
    blocks = generate_delivery_instructions(num_instructions, instructions_per_block)
    
    # Write the generated instructions to a text file, with each block on a single line
    with open(output_file, "w") as file:
        for idx in range(len(blocks)):
            if idx != len(blocks) - 1:
                file.write(blocks[idx] + "\n")  # Add a blank line after each block
            else:
                file.write(blocks[idx])

    print(f"{num_instructions} delivery instructions in blocks of {instructions_per_block} written to '{output_file}'.")

if __name__ == "__main__":
    # Use argparse to parse command-line arguments
    parser = argparse.ArgumentParser(description="Generate delivery instructions and write to a file.")
    parser.add_argument("--num", type=int, required=True, help="Total number of instructions to generate")
    parser.add_argument("--per-block", type=int, required=True, help="Number of instructions per block")
    parser.add_argument("--output-file", type=str, required=True, help="Name of the output file")

    args = parser.parse_args()
    main(args.num, args.per_block, args.output_file)