#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('platoon_control')
    
    # Path to the parameters file
    params_file = os.path.join(pkg_dir, 'params', 'platoon_control.yaml')
    
    # Read the YAML file
    with open(params_file, 'r') as f:
        platoon_configs = yaml.safe_load(f)
    
    # Launch nodes for each platoon configuration
    nodes = []
    
    # Add each platoon configuration
    for config_name, config in platoon_configs.items():
        if not isinstance(config, dict) or 'leader' not in config or 'follower' not in config:
            continue
        
        leader = config['leader']
        follower = config['follower']
        
        # Add a node for this platoon configuration
        node = Node(
            package='platoon_control',
            executable='formation_control_node.py',
            name=f'{leader}_to_{follower}',
            namespace='platoon_control/',
            output='screen',
            parameters=[
                {'leader': leader},
                {'follower': follower}
            ]
        )
        
        nodes.append(node)
    
    return LaunchDescription(nodes)