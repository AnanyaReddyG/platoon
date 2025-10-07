import rclpy
import math
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy

# back_to_lane: after overtaking the obstacle bot attempts to return back to the previous original lane along with goal navigation.

class AdaptivePathPlanner(Node):
    def __init__(self):
        super().__init__("path_planner")
        
        # Publisher to control robot velocity commands (linear and angular)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # QoS Profile to ensure compatibility and reliability for laser scan subscription
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT  # Best effort delivery for sensor messages
        
        # Subscriber for laser scan data to detect obstacles
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        
        # Subscriber for odometry data to track current position and orientation
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Goal location coordinates and thresholds
        self.goal_x = 4.5  # Target goal position X 
        self.goal_y = 0.0  # Target goal position Y 
        self.goal_threshold = 0.1  # Distance threshold to consider goal reached 
        
        # Proportional gains for control (tunable)
        self.linear_kp = 0.8   # Linear velocity gain
        self.angular_kp = 2.0  # Angular velocity gain
        
        self.obstacle_threshold = 0.5  # Minimum safe distance to obstacle 
        
        # Internal states to store sensor and position info
        self.obstacle_detected = False  # Flag if an obstacle detected in path
        self.count = 0  # Counter, used in obstacle avoidance logic
        
        self.current_x = 0.0  # Robot's current x-position
        self.current_y = 0.0  # Robot's current y-position
        self.current_yaw = 0.0  # Robot's current heading (yaw in radians)
        
        self.goal_reached = False  # Flag indicating if goal has been reached
    
    def scan_callback(self, msg):
        """
        Callback to process incoming laser scan data.
        Divides scan data into segments and detects obstacles based on proximity thresholds.
        """
        timestamp = self.get_clock().now().to_msg().sec
        
        # Compute indices corresponding to key angles in laser scan
        index_0_deg = int((0 - msg.angle_min) / msg.angle_increment)
        index_90_deg = int((math.radians(90) - msg.angle_min) / msg.angle_increment)
        index_180_deg = int((math.radians(180) - msg.angle_min) / msg.angle_increment)
        index_270_deg = int((math.radians(270) - msg.angle_min) / msg.angle_increment)
        index_360_deg = int((math.radians(360) - msg.angle_min) / msg.angle_increment)
        
        # Extract laser ranges split by quadrant for analysis
        self.ranges_0_to_90 = msg.ranges[index_0_deg:index_90_deg + 1]
        self.ranges_90_to_180 = msg.ranges[index_90_deg:index_180_deg + 1]
        self.ranges_180_to_270 = msg.ranges[index_180_deg:index_270_deg + 1]
        self.ranges_270_to_360 = msg.ranges[index_270_deg:index_360_deg + 1]
        
        # Full list of ranges for reference
        self.all_ranges = list(msg.ranges)
        self.get_logger().info(f"all_ranges= {len(self.all_ranges)}")
        
        # Determine ranges on the left and right relative to robot's heading and yaw
        self.road_left_view = [i for i, dist in enumerate(self.all_ranges[:int(len(self.all_ranges)//4) - int(self.current_yaw*180/math.pi)]) if dist <= 0.5]
        self.road_right_view = [i for i, dist in enumerate(self.all_ranges[-(int(len(self.all_ranges)//4) + int(self.current_yaw*180/math.pi)):]) if dist <= 0.5]
        
        self.road_block = 1  # Default to blocked
        
        self.get_logger().info(f"road_left={self.road_left_view}")
        self.get_logger().info(f"road_right={self.road_right_view}")
        
        # If no obstacles detected on either side, mark road_block as false
        if len(self.road_left_view) == 0 and len(self.road_right_view) == 0:
            self.road_block = 0
        
        # Set obstacle presence flag based on detected nearby distances
        self.obstacle_detected = self.road_block
        
        self.get_logger().info(f"road_block: {self.road_block}")
        self.get_logger().info(f"obstacle_detected: {self.obstacle_detected}")
    
    def odom_callback(self, msg):
        """
        Callback to update robot's current position and orientation using odometry messages.
        Also checks if the robot has reached its goal.
        """
        # Update current position from odometry
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract yaw angle from quaternion orientation
        quat = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        # Calculate distance to goal
        distance_to_goal = math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)
        self.get_logger().info(f"dist_to_goal: {distance_to_goal}")
        
        # If within threshold distance, stop robot and set goal reached flag
        if distance_to_goal <= self.goal_threshold:
            self.get_logger().info(f"Case #Reached Goal")
            self.goal_reached = True
            self.stop_robot()
            self.get_logger().info("Goal reached!")
            return
        
        # Continue moving toward goal if not reached yet
        if not self.goal_reached:
            self.move_toward_goal(distance_to_goal)
    
    def move_toward_goal(self, distance_to_goal):
        """
        Logic to compute velocity commands to move towards the goal.
        Adapt path if obstacle is detected.
        """
        # Desired yaw pointing straight along y=0 line (lane center)
        required_yaw = (0.0 - self.current_y) * (math.pi / 2)
        
        # Compute shortest angular difference to required heading
        yaw_error = math.atan2(math.sin(required_yaw - self.current_yaw), math.cos(required_yaw - self.current_yaw))
        
        self.get_logger().info(f"Curent yaw: {self.current_yaw}")
        self.get_logger().info(f"odomdata: {self.current_x:.2f}, {self.current_y:.2f}")
        self.get_logger().info(f"obstacle_detected:{self.obstacle_detected}")
        
        if self.obstacle_detected:
            # Obstacle detected: adapt the path planning and movement
            
            self.get_logger().info("Case #0 - Obstacle detected, adapting path...")
            
            # Determine clearance on left and right based on laser scan segments
            self.left_clearance = [i for i, dist in enumerate(self.all_ranges[:int(len(self.all_ranges)//4) - int(self.current_yaw*180/math.pi)]) if dist <= 1.0]
            self.right_clearance = [len(self.all_ranges) // 4 + int(self.current_yaw*180/math.pi) - i - 1 for i, dist in enumerate(self.all_ranges[-(int(len(self.all_ranges)//4) + int(self.current_yaw*180/math.pi)):]) if dist <= 1.0]
            
            self.get_logger().info(f"right_clearance={self.right_clearance}")
            self.get_logger().info(f"left_clearance={self.left_clearance}")
            
            # Determine the left and right clearance angles
            self.left_deg = self.left_clearance[-1] if self.left_clearance else 0
            self.right_deg = self.right_clearance[0] if self.right_clearance else 0
            
            self.get_logger().info(f"left_deg = {self.left_deg} right_deg = {self.right_deg}")
            
            # Choosing side for avoidance based on clearance
            if self.left_deg <= self.right_deg:
                self.get_logger().info(f"Case #1 - Choose left side avoidance")
                if self.left_deg != 0:
                    self.get_logger().info(f"Case #2 - Rotating left to avoid obstacle")
                    self.publish_velocity(0.0, 0.8)  # Rotate left
                else:
                    # If no left clearance, keep moving forward and fine tune
                    self.get_logger().info(f"Case #3 - Moving forward and fine tuning")
                    # Check forward distance before moving forward
                    forward_distance_index = int(3 * len(self.all_ranges) / 4) - int((len(self.all_ranges) * int(self.current_yaw*180/math.pi)) / 360)
                    if self.all_ranges[forward_distance_index] >= 2.0:
                        self.get_logger().info(f"Case #4 - Clear path ahead, moving forward")
                        if self.count < 10:
                            self.get_logger().info(f"Case #5 - Small rotation before forward movement")
                            self.publish_velocity(0.0, 0.8)
                            self.count += 1
                        else:
                            self.publish_velocity(0.1, 0.0)   # Move forward along the obstacle
                    else:
                        self.get_logger().info(f"Case #7 - Fine tune left side orientation")
                        if self.current_yaw <= 0:
                            self.get_logger().info(f"Case #8 - Continue forward fine tuning")
                            self.publish_velocity(0.1, 0.0)
                        else:
                            self.get_logger().info(f"Case #9 - Rotate to adjust left side orientation")
                            self.publish_velocity(0.0, -0.8)
            else:
                # Obstacle avoidance to the right side
                self.get_logger().info(f"Case #1R - Choose right side avoidance")
                if self.right_deg != 0:
                    self.get_logger().info(f"Case #2R - Rotating right to avoid obstacle")
                    self.publish_velocity(0.0, -0.8)  # Rotate right
                else:
                    self.get_logger().info(f"Case #3R - Moving forward and fine tuning")
                    forward_distance_index = int(len(self.all_ranges) / 4) - int((len(self.all_ranges) * int(self.current_yaw*180/math.pi)) / 360)
                    if self.all_ranges[forward_distance_index] >= 2.0:
                        self.get_logger().info(f"Case #4R - Clear path ahead, moving forward")
                        if self.count < 10:
                            self.get_logger().info(f"Case #5R - Small rotation before forward movement")
                            self.publish_velocity(0.0, -0.8)
                            self.count += 1
                        else:
                            self.publish_velocity(0.1, 0.0)  # Move forward
                    else:
                        self.get_logger().info(f"Case #7R - Fine tune right side orientation")
                        if self.current_yaw >= 0:
                            self.get_logger().info(f"Case #8R - Continue forward fine tuning")
                            self.publish_velocity(0.1, 0.0)
                        else:
                            self.get_logger().info(f"Case #9R - Rotate to adjust right side orientation")
                            self.publish_velocity(0.0, 0.8)
        else:
            # No obstacle detected; move straightforwardly towards the goal
            self.get_logger().info(f"Case #100 - No obstacle, moving to goal")
            
            self.count = 0
            linear_speed = min(self.linear_kp * distance_to_goal, 0.1)
            angular_speed = max(-0.1, min(self.angular_kp * yaw_error, 0.1))
            self.publish_velocity(linear_speed, angular_speed)
    
    def publish_velocity(self, speed, turn):
        """
        Publish velocity commands to the robot.
        speed: linear velocity 
        turn: angular velocity 
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
