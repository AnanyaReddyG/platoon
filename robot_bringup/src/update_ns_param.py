import os
import sys

def replace_namespace_in_file(file_path, new_namespace):
    """
    Replace the old namespace (detected as the first key before a colon ':') 
    with the specified new namespace in the YAML file.
    """
    with open(file_path, 'r') as file:
        lines = file.readlines()

    # Detect the old namespace (first non-empty line before a colon)
    old_namespace = None
    for line in lines:
        stripped_line = line.strip()
        if ':' in stripped_line:  # Look for the first colon
            old_namespace = stripped_line.split(':', 1)[0].strip()
            break

    if not old_namespace:
        raise ValueError("Could not detect an old namespace in the YAML file.")

    print(f"Detected old namespace: {old_namespace}")
    print(f"Replacing with new namespace: {new_namespace}")

    # Replace the old namespace with the new namespace in all occurrences
    updated_lines = [line.replace(old_namespace, new_namespace) for line in lines]

    # Write back the updated content to the file
    with open(file_path, 'w') as file:
        file.writelines(updated_lines)

if __name__ == "__main__":

    # Get Turtlebot3 model environment variable for specifying yaml file
    robot_model = os.environ['TURTLEBOT3_MODEL']

    # Default path to the parameter file
    default_param_path = os.path.join(
        os.path.dirname(__file__),    # Path of the current script
        f'../param/{robot_model}.yaml'   # Relative path to the yaml file of robot_model
    )

    # Get new namespace from the environment or use a default
    new_namespace = os.environ.get('TURTLEBOT3_NAMESPACE', 'default_ns')

    # Command-line argument for a custom file path
    param_path = sys.argv[1] if len(sys.argv) > 1 else default_param_path

    print(f"Updating namespace in file: {param_path}")
    
    # Perform the replacement
    replace_namespace_in_file(param_path, new_namespace)
    
    print(f"Successfully updated the {robot_model}.yaml file with {new_namespace} namespace!")