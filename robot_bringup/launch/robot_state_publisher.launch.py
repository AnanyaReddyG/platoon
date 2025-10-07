import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Get environment values from the system
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    ROBOT_NAMESPACE = os.environ.get('TURTLEBOT3_NAMESPACE', 'default_ns')  # namespace for the robot

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'

    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        urdf_file_name)

    # changing the base_footprint frame id to include namespace in it
    with open(urdf, 'r') as infp:
        robot_desc = infp.read().replace(
                            'base_footprint', f'{ROBOT_NAMESPACE}/base_footprint'
                        ).replace(
                            'base_link', f'{ROBOT_NAMESPACE}/base_link'
                        ).replace(
                            'wheel_left_link', f'{ROBOT_NAMESPACE}/wheel_left_link'  
                        ).replace(
                            'wheel_right_link', f'{ROBOT_NAMESPACE}/wheel_right_link'
                        ).replace(
                            'caster_back_link', f'{ROBOT_NAMESPACE}/caster_back_link'
                        ).replace(
                            'imu_link', f'{ROBOT_NAMESPACE}/imu_link'
                        ).replace(
                            'base_scan', f'{ROBOT_NAMESPACE}/base_scan'
                        )

    rsp_params = {'robot_description': robot_desc}
    
    # print (robot_desc) # Printing urdf information.

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ROBOT_NAMESPACE,  # <--- CHANGE THIS to your desired robot namespace
            output='screen',
            parameters=[rsp_params, {'use_sim_time': use_sim_time}])
    ])
