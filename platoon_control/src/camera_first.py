import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseArray

# camera_first: getting postions of obstacles using ArUco markers instead of scan data.

class AdaptivePathPlanner(Node):
    def __init__(self):
        super().__init__("camera_first")
        
        # Publisher for robot velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        
        # Subscriber for laser scan data with compatible QoS
        # self.scan_subscription = self.create_subscription(
        #     LaserScan, 
        #     '/burger3/scan', 
        #     self.scan_callback, 
        #     qos_profile  # Apply the QoS profile
        # )

        # Subscribe to estimated aruco marker poses
        self.poses_subscription = self.create_subscription(
            PoseArray,
            '/burger3/aruco_poses',
            self.poses_callback,
            qos_profile
        )
        # Subscribe to detected aruco marker IDs
        self.ids_subscription = self.create_subscription(
            Int32MultiArray,
            '/burger3/aruco_ids',
            self.ids_callback,
            qos_profile
        )
        # Subscribe to robot odometry
        self.dummy = 12
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # The goal is initialized 
        self.goal_x = 1.1
        self.goal_y = 15.0
        self.goal_threshold = 0.1
        self.linear_kp = 0.4
        self.angular_kp = 2.0
        self.obstacle_threshold = 0.7 # Safe distance to consider as obstacle detected
        
        self.obstacle_detected = False
        self.count = 0
        
        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.goal_reached = False
        self.obst_x = 10.0
        self.obst_y = 10.0
        self.obst_lane = 10.0
        self.bot_lane = 10.0
        self.detected_ids = []

    def poses_callback(self, msg):
        # Called with array of aruco marker/robot positions
        self.aruco_detected = False
        self.closest_aruco_y = None
        if not msg.poses:
            return

        if len(self.detected_ids) != len(msg.poses):
            return
        for i in range(len(self.detected_ids)):
            if self.detected_ids[i] == 0:
                # ID 0 is assumed to be the bot
                self.current_x = msg.poses[i].position.x
                self.current_y = msg.poses[i].position.y
            else:
                # Nonzero ID, assume it's the obstacle
                self.obst_x = msg.poses[i].position.x
                self.obst_y = msg.poses[i].position.y
        # Lane assignment based on y-position relative to goal line
        if self.goal_y == 15.0:
            self.goal_y = self.current_y
        if self.current_y < -0.15 + self.goal_y:
            self.bot_lane = 0
        elif self.current_y < 0.15 + self.goal_y:
            self.bot_lane = 1
        else:
            self.bot_lane = 2
        if self.obst_y < -0.15 + self.goal_y:
            self.obst_lane = 0
        elif self.obst_y < 0.15 + self.goal_y:
            self.obst_lane = 1
        else:
            self.obst_lane = 2
        # Determine if obstacle detected
        dist_btw_bot_and_obst = (self.obst_x - self.current_x)
        if self.obst_lane == self.bot_lane and dist_btw_bot_and_obst <= self.obstacle_threshold and dist_btw_bot_and_obst > 0:
            self.obstacle_detected = 1
        elif self.current_x >= self.obst_x:
            self.obstacle_detected = 0

    def ids_callback(self, msg):
        # Updates the detected aruco IDs list
        self.detected_ids = msg.data

    def odom_callback(self, msg):
        # Update robot orientation from odometry
        quat = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        distance_to_goal = math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)
        
        # Stop if goal reached
        if distance_to_goal <= self.goal_threshold:
            self.goal_reached = True
            self.stop_robot()
            return
        
        # navigation: if not reached, move towards goal
        if not self.goal_reached:
            self.move_toward_goal(distance_to_goal)

    def move_toward_goal(self, distance_to_goal):
        # Compute required yaw to the goal
        required_yaw = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        yaw_error = math.atan2(math.sin(required_yaw - self.current_yaw), math.cos(required_yaw - self.current_yaw))

        if self.obstacle_detected:
            # If an obstacle is detected in current lane and close ahead,
            # plan to go left or right.
            if self.current_y > self.obst_y: # Overtaking left
                required_yaw = -(0.3 + self.goal_y - self.current_y)*(math.pi/2)
            else: # else right
                required_yaw = -(-0.3 + self.goal_y - self.current_y)*(math.pi/2)
            yaw_error = math.atan2(math.sin(required_yaw - self.current_yaw), math.cos(required_yaw - self.current_yaw))
            linear_speed = min(self.linear_kp * distance_to_goal, 0.1)
            angular_speed = max(-0.8, min(self.angular_kp * yaw_error, 0.8))
            self.publish_velocity(linear_speed, angular_speed)
        else:
            # No obstacle detected, go straight to goal
            required_yaw = -(0.0 + self.goal_y - self.current_y)*(math.pi/2)
            yaw_error = math.atan2(math.sin(required_yaw - self.current_yaw), math.cos(required_yaw - self.current_yaw))
            linear_speed = min(self.linear_kp * distance_to_goal, 0.1)
            angular_speed = max(-0.8, min(self.angular_kp * yaw_error, 0.8))
            self.publish_velocity(linear_speed, angular_speed)

    def publish_velocity(self, speed, turn):
        # Publish a Twist message for velocity control
        vel_msg = Twist()
        vel_msg.linear.x = speed
        vel_msg.angular.z = turn
        self.cmd_vel_pub.publish(vel_msg)

    def stop_robot(self):
        # Stop the robot by publishing zero speed/turn
        self.publish_velocity(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = AdaptivePathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
