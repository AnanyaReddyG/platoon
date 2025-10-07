import rclpy
import math
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy

# lane_changing: if any obstacle is detected in the way of bot it will compleetly move to the left or right lane irrespective of the size of the obstacle unlike back_to_lane and direct_to_goal logics which depend on the shape and size of obstacles.

class AdaptivePathPlanner(Node):
    def __init__(self):
        super().__init__("path_planner_wafflepi1")

        # Publisher for velocity commands (Twist msg)
        self.cmd_vel_pub = self.create_publisher(Twist, "/wafflepi1/cmd_vel", 10)

        # Set QoS profile for subscriptions
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        # Subscribe to laser scan for obstacle detection
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/wafflepi1/scan',
            self.scan_callback,
            qos_profile
        )
        # Subscribe to odometry for pose tracking
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/wafflepi1/odom',
            self.odom_callback,
            10
        )

        # Lane change parameters and initial states
        self.goal_x = 3.0           # Goal X position 
        self.goal_y = 0.0           # Goal Y position 
        self.goal_threshold = 0.1   # Threshold to stop near goal 
        self.linear_kp = 0.8        # Linear speed gain
        self.angular_kp = 2.0       # Angular speed gain
        self.obstacle_threshold = 0.5  # Distance to consider obstacle present
        self.obstacle_detected = False  # True if obstacle is present
        self.count = 0               # Used for turning/counter
        self.lane_width = 0.5        # Assumed lane width

        # current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.goal_reached = False

    def scan_callback(self, msg):
        """
        Called whenever the robot receives new laser scan data.
        Divides the scan into sectors and checks for obstacles in each quadrant.
        """
        timestamp = self.get_clock().now().to_msg().sec

        # Indices corresponding to key angles in scan
        # Calculate indices for 0°, 90°, 270°, and 360°
        index_0_deg = int((0 - msg.angle_min) / msg.angle_increment)
        index_90_deg = int((math.radians(90) - msg.angle_min) / msg.angle_increment)
        index_180_deg = int((math.radians(180) - msg.angle_min) / msg.angle_increment)
        index_270_deg = int((math.radians(270) - msg.angle_min) / msg.angle_increment)
        index_360_deg = int((math.radians(360) - msg.angle_min) / msg.angle_increment)

        # Slice scan data into relevant sectors/quadrants
        self.ranges_0_to_90 = msg.ranges[index_0_deg:index_90_deg + 1]
        self.ranges_90_to_180 = msg.ranges[index_90_deg:index_180_deg + 1]
        self.ranges_180_to_270 = msg.ranges[index_180_deg:index_270_deg + 1]
        self.ranges_270_to_360 = msg.ranges[index_270_deg:index_360_deg + 1]

        self.all_ranges = list(msg.ranges)
        self.get_logger().info(f"all_ranges= {len(self.all_ranges)}")

        # Check sectors for obstacles close to robot (within 70cm)
        self.road_left_view = [i for i, dist in enumerate(self.all_ranges[:int(len(self.all_ranges)//4) - int(self.current_yaw*len(self.all_ranges)/(2*math.pi))]) if dist <= 0.7]
        self.road_right_view = [i for i, dist in enumerate(self.all_ranges[-(int(len(self.all_ranges)//4) + int(self.current_yaw*len(self.all_ranges)/(2*math.pi))):]) if dist <= 0.7]

        self.road_block = 1  # Assume blocked unless both views are clear
        self.get_logger().info(f"road_left={self.road_left_view}")
        self.get_logger().info(f"road_right={self.road_right_view}")

        # No obstacle blocking if both views are empty
        if len(self.road_left_view) == 0 and len(self.road_right_view) == 0:
            self.road_block = 0

        # Set obstacle presence flag for later use in maneuvers
        self.obstacle_detected = self.road_block
        self.get_logger().info(f"road_block: {self.road_block}")
        self.get_logger().info(f"obstacle_detected: {self.obstacle_detected}")

    def odom_callback(self, msg):
        """
        Called whenever the robot receives odometry data about its pose.
        Updates position, orientation, and determines if goal is reached.
        """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        # Check distance to goal
        distance_to_goal = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
        self.get_logger().info(f"dist_to_goal: {distance_to_goal}")

        # If at the goal, stop robot
        if distance_to_goal <= self.goal_threshold:
            self.get_logger().info("Case #Reached Goal")
            self.goal_reached = True
            self.stop_robot()
            self.get_logger().info("Goal reached!")
            return

        # Otherwise, continue lane change or navigation
        if not self.goal_reached:
            self.move_toward_goal(distance_to_goal)

    def move_toward_goal(self, distance_to_goal):
        """
        Computes velocity command based on current goal and obstacle observations.
        Handles logic for moving straight, or steering left/right to change lanes as needed.
        """
        # Heading required to move toward goal
        required_yaw = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        yaw_error = math.atan2(math.sin(required_yaw - self.current_yaw), math.cos(required_yaw - self.current_yaw))

        self.get_logger().info(f"Curent yaw: {self.current_yaw}")
        self.get_logger().info(f"odomdata: {self.current_x:.2f}, {self.current_y:.2f}")

        # Obstacle avoidance and lane changing logic
        if self.obstacle_detected:
            # Determine clearance for left and right lanes
            self.left_clearance = [i for i, dist in enumerate(self.all_ranges[:int(len(self.all_ranges)//4) - int(self.current_yaw*len(self.all_ranges)/(2*math.pi))]) if dist <= 1.0]
            self.right_clearance = [len(self.all_ranges) // 4 + int(self.current_yaw*len(self.all_ranges)/(2*math.pi)) - i - 1 for i, dist in enumerate(self.all_ranges[-(int(len(self.all_ranges)//4) + int(self.current_yaw*len(self.all_ranges)/(2*math.pi))):]) if dist <= 1.0]
            self.get_logger().info(f"right_clearance={self.right_clearance}")
            self.get_logger().info(f"left_clearance={self.left_clearance}")

            self.left_deg = self.left_clearance[-1] if self.left_clearance else 0
            self.right_deg = self.right_clearance[0] if self.right_clearance else 0
            self.get_logger().info(f"left_deg = {self.left_deg} right_deg = {self.right_deg}")

            # Favor side with most clearance for lane change
            if self.left_deg <= self.right_deg:
                # Move left 
                required_yaw = (self.lane_width - self.current_y) * (math.pi/2)
                yaw_error = math.atan2(math.sin(required_yaw - self.current_yaw), math.cos(required_yaw - self.current_yaw))
                self.get_logger().info(f"Case #1")
                self.get_logger().info(f"required_yaw: {required_yaw}")
                self.get_logger().info(f"current_yaw: {self.current_yaw}")
                self.get_logger().info(f"yaw_error: {yaw_error}")
                linear_speed = min(self.linear_kp * distance_to_goal, 0.1)
                angular_speed = max(-0.8, min(self.angular_kp * yaw_error, 0.8))
                self.get_logger().info(f"angular_speed: {angular_speed}")
                self.get_logger().info(f"linear_vel: {linear_speed}")
                self.publish_velocity(linear_speed, angular_speed)

            else:
                # Move right 
                self.get_logger().info(f"Case #1R")
                required_yaw = (-self.lane_width - self.current_y) * (math.pi/2)
                self.get_logger().info(f"required_yaw: {required_yaw}")
                self.get_logger().info(f"current_yaw: {self.current_yaw}")
                yaw_error = math.atan2(math.sin(required_yaw - self.current_yaw), math.cos(required_yaw - self.current_yaw))
                linear_speed = min(self.linear_kp * distance_to_goal, 0.1)
                self.get_logger().info(f"yaw_error: {yaw_error}")
                angular_speed = max(-0.8, min(self.angular_kp * yaw_error, 0.8))
                self.get_logger().info(f"angular_speed: {angular_speed}")
                self.get_logger().info(f"linear_vel: {linear_speed}")
                self.publish_velocity(linear_speed, angular_speed)
        else:
            # No obstacle: proceed along center or lane as normal
            self.get_logger().info("Case #100")
            required_yaw = (0.0 - self.current_y) * (math.pi/2)
            yaw_error = math.atan2(math.sin(required_yaw - self.current_yaw), math.cos(required_yaw - self.current_yaw))
            self.get_logger().info(f"required_yaw: {required_yaw}")
            self.get_logger().info(f"yaw_error: {yaw_error}")
            self.get_logger().info(f"current_yaw: {self.current_yaw}")
            linear_speed = min(self.linear_kp * distance_to_goal, 0.1)
            angular_speed = max(-0.8, min(self.angular_kp * yaw_error, 0.8))
            self.get_logger().info(f"angular_speed: {angular_speed}")
            self.get_logger().info(f"linear_vel: {linear_speed}")
            self.publish_velocity(linear_speed, angular_speed)

    def publish_velocity(self, speed, turn):
        """
        Publishes velocity command to /wafflepi1/cmd_vel topic.
        speed: linear velocity (m/s)
        turn: angular velocity (rad/s)
        """
        vel_msg = Twist()
        vel_msg.linear.x = speed
        vel_msg.angular.z = turn
        self.cmd_vel_pub.publish(vel_msg)

    def stop_robot(self):
        """
        Stops the robot by publishing zero velocities.
        """
        self.publish_velocity(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = AdaptivePathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
