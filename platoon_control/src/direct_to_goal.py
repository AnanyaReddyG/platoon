import rclpy
import math
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy

# direct_to_goal: after overtaking the obstacle the bot does not attempt to return to its original lane. It directly targets goal.

class AdaptivePathPlanner(Node):
    def __init__(self):
        super().__init__("path_planner")
        
        # Publisher to control robot movement (velocity and turning)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # QoS Profile for reliable and compatible sensor data subscription
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        
        # Subscriber for laser scan data (used for obstacle detection)
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        
        # Subscriber for odometry (current pose and orientation)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Goal position and control tuning
        self.goal_x = 7.0   # Target X position 
        self.goal_y = 0.0   # Target Y position 
        self.goal_threshold = 0.1  # Distance threshold for "goal reached" 
        self.linear_kp = 0.8        # Linear velocity gain
        self.angular_kp = 2.0       # Angular velocity gain
        self.obstacle_threshold = 1.0  # Min distance before obstacle is "detected" 
        
        # Internal state variables
        self.obstacle_detected = False  # True if obstacle blocks the road ahead
        self.count = 0                  # Used for counting iterative maneuvers
        
        # Robot's current position and orientation variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        self.goal_reached = False  # True once target is reached
        
    def scan_callback(self, msg):
        """
        Called whenever new laser scan data arrives.
        Segments the scan into quadrants and detects obstacles based on close-range indices.
        """
        timestamp = self.get_clock().now().to_msg().sec
        
        # Index calculation for various angles (based on message config)
        index_0_deg = int((0 - msg.angle_min) / msg.angle_increment)
        index_90_deg = int((math.radians(90) - msg.angle_min) / msg.angle_increment)
        index_180_deg = int((math.radians(180) - msg.angle_min) / msg.angle_increment)
        index_270_deg = int((math.radians(270) - msg.angle_min) / msg.angle_increment)
        index_360_deg = int((math.radians(360) - msg.angle_min) / msg.angle_increment)
        
        # Split ranges into different quadrants for analysis
        self.ranges_0_to_90 = msg.ranges[index_0_deg:index_90_deg + 1]
        self.ranges_90_to_180 = msg.ranges[index_90_deg:index_180_deg + 1]
        self.ranges_180_to_270 = msg.ranges[index_180_deg:index_270_deg + 1]
        self.ranges_270_to_360 = msg.ranges[index_270_deg:index_360_deg + 1]
        
        self.all_ranges = list(msg.ranges)  # All LIDAR ranges
        self.get_logger().info(f"all_ranges= {len(self.all_ranges)}")
        
        # Find obstacles within 1.0m in left and right quadrants
        self.road_left_view = [i for i, dist in enumerate(
            self.all_ranges[:int(len(self.all_ranges)//4) - int(self.current_yaw*180/math.pi)]
        ) if dist <= 1.0]
        self.road_right_view = [i for i, dist in enumerate(
            self.all_ranges[-(int(len(self.all_ranges)//4) + int(self.current_yaw*180/math.pi)):]
        ) if dist <= 1.0]
        self.road_block = 1 # Default to blocked
        
        self.get_logger().info(f"road_left={self.road_left_view}")
        self.get_logger().info(f"road_right={self.road_right_view}")
        
        # If both sides are clear, mark road as unblocked
        if len(self.road_left_view) == 0 and len(self.road_right_view) == 0:
            self.road_block = 0
        
        # Flag whether obstacle detected, based on road block state
        self.obstacle_detected = self.road_block
        
        self.get_logger().info(f"road_block: {self.road_block}")
        self.get_logger().info(f"obstacle_detected: {self.obstacle_detected}")
    
    def odom_callback(self, msg):
        """
        Called whenever new odometry data arrives.
        Updates robot's pose and orientation, then decides next movement.
        """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        quat = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        # Compute distance to target
        distance_to_goal = math.sqrt(
            (self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2
        )
        self.get_logger().info(f"dist_to_goal: {distance_to_goal}")
        
        # If goal reached, stop movement
        if distance_to_goal <= self.goal_threshold:
            self.get_logger().info("Case #Reached Goal")
            self.goal_reached = True
            self.stop_robot()
            self.get_logger().info("Goal reached!")
            return
        
        # Otherwise, attempt to move towards goal
        if not self.goal_reached:
            self.move_toward_goal(distance_to_goal)
    
    def move_toward_goal(self, distance_to_goal):
        """
        Main logic to move robot towards goal, adapting if obstacles are seen.
        Decides on avoidance direction and motion depending on sensor inputs.
        """
        # Desired heading towards goal
        required_yaw = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        # Angular error between current heading and desired heading
        yaw_error = math.atan2(math.sin(required_yaw - self.current_yaw), math.cos(required_yaw - self.current_yaw))
        
        self.get_logger().info(f"Curent yaw: {self.current_yaw}")
        self.get_logger().info(f"odomdata: {self.current_x:.2f}, {self.current_y:.2f}")
        
        if self.obstacle_detected:
            # Obstacle detected: need to pick avoidance direction, maneuver
            
            self.get_logger().info("Case #0")
            self.get_logger().info("Obstacle detected, adapting path...")
            
            # Find clearance indices for both sides
            self.left_clearance = [i for i, dist in enumerate(
                self.all_ranges[:int(len(self.all_ranges)//4) - int(self.current_yaw*180/math.pi)]
            ) if dist <= 1.0]
            self.right_clearance = [len(self.all_ranges) // 4 + int(self.current_yaw*180/math.pi) - i - 1 for i, dist in enumerate(
                self.all_ranges[-(int(len(self.all_ranges)//4) + int(self.current_yaw*180/math.pi)):]
            ) if dist <= 1.0]
            
            self.get_logger().info(f"right_clearance={self.right_clearance}")
            self.get_logger().info(f"left_clearance={self.left_clearance}")
            
            # Find the most open angle in left/right clearance arrays
            self.left_deg = self.left_clearance[-1] if self.left_clearance else 0
            self.right_deg = self.right_clearance[0] if self.right_clearance else 0
            self.get_logger().info(f"left_deg = {self.left_deg} right_deg = {self.right_deg}")
            
            # Choose left or right based on which side is more clear
            if self.left_deg <= self.right_deg:
                # Favor left side
                self.get_logger().info("Case #1")
                if self.left_deg != 0:
                    self.get_logger().info("Case #2")
                    self.get_logger().info("rotating left")
                    self.get_logger().info(f"dist == {self.all_ranges[0-int(self.current_yaw*180/math.pi)]}")
                    self.get_logger().info(f"current_yaw_in_deg = {int(self.current_yaw*180/math.pi)}")
                    # Turn robot left
                    self.publish_velocity(0.0, 0.8)
                else:
                    self.get_logger().info("Case #3")
                    forward_index = int(3 * len(self.all_ranges) / 4) - int((len(self.all_ranges) * int(self.current_yaw*180/math.pi)) / 360)
                    self.get_logger().info(f"angle:{forward_index}")
                    self.get_logger().info(f"distance:{self.all_ranges[forward_index]}")
                    # Move forward if space ahead, otherwise rotate or fine tune
                    if self.all_ranges[forward_index] >= 2.0:
                        self.get_logger().info("Case #4")
                        if self.count < 10:
                            self.get_logger().info("Case #5")
                            self.publish_velocity(0.0, 0.8)
                            self.count += 1
                        else:
                            self.get_logger().info("Case #6")
                            self.publish_velocity(0.1, 0.0)
                    else:
                        self.get_logger().info("Case #7")
                        self.get_logger().info("turning")
                        # Fine tune orientation if not enough clearance
                        if self.current_yaw <= 0:
                            self.get_logger().info("Case #8")
                            self.publish_velocity(0.1, 0.0)
                        else:
                            self.get_logger().info("Case #9")
                            self.publish_velocity(0.0, -0.8)
            else:
                # Favor right side
                self.get_logger().info("Case #1R")
                if self.right_deg != 0:
                    self.get_logger().info("Case #2R")
                    self.get_logger().info("rotating right")
                    self.get_logger().info(f"dist == {self.all_ranges[0-int(self.current_yaw*180/math.pi)]}")
                    self.get_logger().info(f"current_yaw_in_deg = {int(self.current_yaw*180/math.pi)}")
                    self.publish_velocity(0.0, -0.8)
                else:
                    self.get_logger().info("Case #3R")
                    forward_index = int(len(self.all_ranges) / 4) - int((len(self.all_ranges) * int(self.current_yaw*180/math.pi)) / 360)
                    self.get_logger().info(f"angle:{forward_index}")
                    self.get_logger().info(f"distance:{self.all_ranges[forward_index]}")
                    if self.all_ranges[forward_index] >= 2.0:
                        self.get_logger().info("Case #4R")
                        if self.count < 10:
                            self.get_logger().info("Case #5R")
                            self.publish_velocity(0.0, -0.8)
                            self.count += 1
                        else:
                            self.get_logger().info("Case #6R")
                            self.publish_velocity(0.1, 0.0)
                    else:
                        self.get_logger().info("Case #7R")
                        self.get_logger().info("turning")
                        if self.current_yaw >= 0:
                            self.get_logger().info("Case #8R")
                            self.publish_velocity(0.1, 0.0)
                        else:
                            self.get_logger().info("Case #9R")
                            self.publish_velocity(0.0, 0.8)
        else:
            # No obstacle: proceed directly toward goal using proportional control
            self.get_logger().info("Case #100")
            self.count = 0
            linear_speed = min(self.linear_kp * distance_to_goal, 0.2)
            angular_speed = max(-0.1, min(self.angular_kp * yaw_error, 0.1))
            self.publish_velocity(linear_speed, angular_speed)
    
    def publish_velocity(self, speed, turn):
        """
        Publishes velocity command to /cmd_vel topic.
        speed: linear velocity 
        turn: angular velocity 
        """
        vel_msg = Twist()
        vel_msg.linear.x = speed
        vel_msg.angular.z = turn
        self.cmd_vel_pub.publish(vel_msg)
    
    def stop_robot(self):
        """
        Stops robot by publishing zero velocity and turn.
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
