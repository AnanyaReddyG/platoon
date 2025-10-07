import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():

    # Get the environment values from the system
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']     # burger or waffle_pi
    LDS_MODEL = os.environ['LDS_MODEL']                   # LDS-01 or LDS-02
    ROBOT_NAMESPACE = os.environ.get('TURTLEBOT3_NAMESPACE', 'default_ns')  # name for your robot

    try:
        if LDS_MODEL == 'LDS-02':
            LDS_LAUNCH_FILE = '/ld08.launch.py'
    except:
        print('Please make sure you have correct LDS_MODEL (LDS-02)!')

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    if ROBOT_NAMESPACE != 'default_ns':
        print(f'Using user defined namespace : {ROBOT_NAMESPACE}')
    else:
        print('Using default namespace : default_ns')

    tb3_param_dir = LaunchConfiguration(
        'tb3_param_dir',
        default=os.path.join(
            get_package_share_directory('robot_bringup'),
            'param',
            TURTLEBOT3_MODEL + '.yaml'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OpenCR'),

        DeclareLaunchArgument(
            'tb3_param_dir',
            default_value=tb3_param_dir,
            description='Full path to turtlebot3 parameter file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), LDS_LAUNCH_FILE]),
            launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}.items(),
        ),

        # This node starts all the sensors and initializes odometry & diff drive controller
        Node(
            package='updated_turtlebot3_node',    # an updated version which allows us to initialize odometry
            executable='updated_turtlebot3_ros',
            namespace=ROBOT_NAMESPACE,      # <--- CHANGE THIS to your desired robot namespace
            parameters=[tb3_param_dir],
            arguments=['-i', usb_port],
            output='screen'),

        # # This node publishes robot trajectory which can visualized in rviz
        # Node(
        #     package='robot_bringup',
        #     executable='robot_trajectory_node.py',
        #     namespace=ROBOT_NAMESPACE,
        #     remappings=[
        #         ('robot_trajectory', f'/{ROBOT_NAMESPACE}/robot_trajectory'),
        #         ('odom', f'/{ROBOT_NAMESPACE}/odom'),
        #     ]
        # )
    ])
