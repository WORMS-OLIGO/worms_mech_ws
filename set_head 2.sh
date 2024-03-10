#!/bin/bash

# Prompt the user for input
echo "Please enter the string you want to write to head.txt:"
read user_input

# Define the file path
file_path="src/worms_mech/worms_mech/head.txt"

# Write the input to head.txt, overwriting the file if it exists
echo "$user_input" > "$file_path"

echo "The file has been created/updated at $file_path"

# Navigate to the root of your workspace (if not already there)
# cd /path/to/your/workspace

# Run colcon build
colcon build

# Source the install/setup.bash script
source install/setup.bash

