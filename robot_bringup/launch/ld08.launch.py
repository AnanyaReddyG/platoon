import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ROBOT_NAMESPACE = os.environ.get('TURTLEBOT3_NAMESPACE', 'default_ns')  # name for the robot

    return LaunchDescription([
        Node(
            package='ld08_driver',
            executable='ld08_driver',
            name='ld08_driver',
            namespace=ROBOT_NAMESPACE,  # <--- CHANGE THIS to your desired robot namespace
            output='screen'),
    ])
