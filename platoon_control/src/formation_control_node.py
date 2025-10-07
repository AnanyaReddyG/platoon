#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
from collections import deque
from tf_transformations import euler_from_quaternion
import numpy as np
import signal
from std_msgs.msg import Float64

MAX_LIN_VEL = 0.22
MAX_ANG_VEL = 2.84

class PlatoonControl(Node):
    def __init__(self):
        super().__init__('platoon_control')
        
        # Declare parameters for leader and follower names
        self.declare_parameter('leader', 'waffle1')  # Default leader name is 'waffle1'
        self.declare_parameter('follower', 'waffle2')  # Default follower name is 'waffle2'
        
        # Get the parameters
        self.leader = self.get_parameter('leader').get_parameter_value().string_value
        self.follower = self.get_parameter('follower').get_parameter_value().string_value

        self.odom_sub_leader = self.create_subscription(Odometry, f'/{self.leader}/odom', self.leader_odom_callback, 10)
        self.odom_sub_follower = self.create_subscription(Odometry, f'/{self.follower}/odom', self.follower_odom_callback, 10)
        self.cmd_pub_follower = self.create_publisher(Twist, f'/{self.follower}/cmd_vel', 10)
        self.leader_vel_sub = self.create_subscription(Twist, f'/{self.leader}/cmd_vel', self.leader_vel_callback, 10)
        self.plt_curr_dist_pub = self.create_publisher(
            Float64,
            f'/platoon_control/{self.leader}_to_{self.follower}/current_distance',
            10
        )
        
        # self.plt_des_dist_pub = self.create_publisher(
        #     Float64,
        #     f'/platoon_control/{self.leader}_to_{self.follower}/desired_distance',
        #     10
        # )
        
        self.leader_moving = False
        self.leader_traj = deque(maxlen=1000)  # Stores leader's trajectory with limited size
        self.follower_pos = None  # Stores current position of follower
        self.leader_pos = None  # Stores current position of leader
        self.leader_yaw = None  # Stores current yaw of leader
        self.follower_yaw = None  # Stores current yaw of follower
        
        self.initial_pos_leader = None  # Initial position of leader
        self.initial_pos_follower = None  # Initial position of follower
        self.last_leader_pos = None  # Track last known position of leader
        
        # Original PD Controller for Trajectory Following
        self.kp_linear = 1.0  # Proportional gain for linear velocity
        self.kd_linear = 0.5  # Derivative gain for linear velocity
        self.kp_angular = 2.5  # Proportional gain for angular velocity
        self.kd_angular = 0.5  # Derivative gain for angular velocity
        
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        
        # PID Controller for Path Distance
        self.kp_distance = 0.9  # Proportional gain for distance control
        self.ki_distance = 0.2  # Integral gain for distance control
        self.kd_distance = 0.5  # Derivative gain for distance control
        self.desired_distance = None  # Desired distance in meters between robots along the path
        self.prev_distance_error = 0.0
        self.integral_distance_error = 0.0
        self.max_integral = 1.0  # Max integral value to prevent windup
        
        # Blending factor between trajectory following and distance control
        self.traj_weight = 0.4  # Weight for trajectory following
        self.dist_weight = 0.6  # Weight for distance control
        
        self.trajectory_initialized = False
        
        # Create a windowed list to calculate path distance
        self.path_window = deque(maxlen=100)  # Store recent path points
        self.total_path_length = 0.0  # Total path length for leader

    def leader_vel_callback(self, msg):
        if abs(msg.linear.x) >= 0.01: # or abs(msg.angular.z) > 0.01:
            self.leader_moving = True


    def leader_odom_callback(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.leader_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        
        self.leader_pos = (x, y)  # Update current position of leader
        
        if self.initial_pos_leader is None:
            self.initial_pos_leader = (x, y)  # Store leader's initial position
            self.last_leader_pos = (x, y)
            self.path_window.append((x, y))
            self.initialize_trajectory_if_ready()
        
        if self.trajectory_initialized and self.distance(x, y, *self.last_leader_pos) > 0.01:
            self.leader_traj.append((x, y))
            
            # Update path window and calculate incremental path length
            path_increment = self.distance(x, y, *self.last_leader_pos)
            self.total_path_length += path_increment
            self.path_window.append((x, y, self.total_path_length))  # Store position and cumulative path length
            
            self.last_leader_pos = (x, y)
            self.get_logger().debug(f"leader path length: {self.total_path_length}")
        
    def follower_odom_callback(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.follower_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        
        if self.initial_pos_follower is None:
            self.initial_pos_follower = (x, y)
            self.initialize_trajectory_if_ready()
        
        self.follower_pos = (x, y)
        
        if self.trajectory_initialized and self.leader_moving:
            self.follow_leader()

    def initialize_trajectory_if_ready(self):
        if self.initial_pos_leader is not None and self.initial_pos_follower is not None:
            self.initialize_trajectory()

    def initialize_trajectory(self):
        xf, yf = self.initial_pos_follower
        xl, yl = self.initial_pos_leader
        self.desired_distance = self.distance(xf, yf, xl, yl)
        # print(f'[Platoon control desired distance initialized to: {self.desired_distance}]')
        
        # # Publishing this desired distance to ros topic
        # msg = Float64()
        # msg.data = self.desired_distance
        # self.plt_des_dist_pub.publish(msg)
        self.get_logger().info(f'Desired distance for platoon control: {self.desired_distance}')

        num_waypoints = int(self.desired_distance * 20)
        initial_waypoints = self.generate_straight_line_waypoints(self.initial_pos_follower, self.initial_pos_leader, num_waypoints)

        # print('[Initial Trajectory waypoints for Follower:]')

        # Calculate cumulative path length for initial trajectory
        cum_length = 0.0
        prev_x, prev_y = self.initial_pos_follower
        
        for i, (x, y) in enumerate(initial_waypoints):
            if i > 0:
                segment_length = self.distance(x, y, prev_x, prev_y)
                cum_length += segment_length
            # print(f"({x}, {y}) - path length: {cum_length}")
            self.leader_traj.append((x, y))
            prev_x, prev_y = x, y
        
        self.trajectory_initialized = True
        self.get_logger().info("[Leader's starting trajectory initialized!]")

    def generate_straight_line_waypoints(self, start, end, num_points):
        x_start, y_start = start
        x_end, y_end = end
        x_points = np.linspace(x_start, x_end, num_points)
        y_points = np.linspace(y_start, y_end, num_points)

        return [(x, y) for x, y in zip(x_points, y_points)]

    def follow_leader(self):
        if len(self.leader_traj) < 2 or self.leader_pos is None or self.follower_pos is None:
            return  # Wait for enough trajectory points and both robot positions
        
        target_x, target_y = self.leader_traj[0]  # Get the earliest stored position
        current_x, current_y = self.follower_pos
        
        # Compute trajectory following command
        traj_cmd = self.compute_trajectory_control(current_x, current_y, target_x, target_y, self.follower_yaw)
        
        # Compute distance control command using path length
        dist_cmd = self.compute_relative_distance_control()
        
        # Blend the two commands
        cmd = Twist()
        cmd.linear.x = self.traj_weight * traj_cmd.linear.x + self.dist_weight * dist_cmd.linear.x
        cmd.angular.z = traj_cmd.angular.z  # Use only trajectory control for angular velocity
        
        # Apply safety limits
        cmd.linear.x = max(0.0, min(MAX_LIN_VEL, cmd.linear.x))  # Limit Linear vel
        cmd.angular.z = max(-MAX_ANG_VEL, min(MAX_ANG_VEL, cmd.angular.z))  # Limit angular vel
        
        self.cmd_pub_follower.publish(cmd)
        
        # Check if we've reached the current target point
        if self.distance(current_x, current_y, target_x, target_y) < 0.1:
            self.leader_traj.popleft()  # Remove old points once reached
    
    def compute_trajectory_control(self, current_x, current_y, target_x, target_y, target_yaw):
        cmd = Twist()
        
        dx = target_x - current_x
        dy = target_y - current_y
        
        desired_yaw = math.atan2(dy, dx)
        
        angular_error = self.normalize_angle(desired_yaw - target_yaw)
        linear_error = math.sqrt(dx**2 + dy**2)
        
        d_linear = linear_error - self.prev_linear_error
        d_angular = angular_error - self.prev_angular_error
        
        cmd.linear.x = self.kp_linear * linear_error + self.kd_linear * d_linear
        cmd.angular.z = self.kp_angular * angular_error + self.kd_angular * d_angular
        
        self.prev_linear_error = linear_error
        self.prev_angular_error = angular_error
        
        return cmd
    
    def compute_relative_distance_control(self):
        """Compute control commands to maintain desired distance from leader along the path"""
        cmd = Twist()
        
        if self.leader_pos is None or self.follower_pos is None or len(self.path_window) < 2:
            return cmd
        
        # Calculate path distance between robots
        path_distance = self.calculate_path_distance_between_robots()
        # print(path_distance)

        # Publishing this current distance to ros topic
        msg = Float64()
        msg.data = path_distance
        self.plt_curr_dist_pub.publish(msg)
        print(f'[Current platoon distance: {path_distance}]')
        
        # Calculate distance error (positive if too far, negative if too close)
        distance_error = path_distance - self.desired_distance
        
        # Calculate derivative term
        d_distance = distance_error - self.prev_distance_error
        
        # Calculate integral term with anti-windup
        self.integral_distance_error += distance_error
        self.integral_distance_error = max(-self.max_integral, min(self.max_integral, self.integral_distance_error))
        
        # PID control for distance
        # If too close (negative error), slow down or go backward
        # If too far (positive error), speed up
        cmd.linear.x = (self.kp_distance * distance_error + 
                       self.ki_distance * self.integral_distance_error +
                       self.kd_distance * d_distance)
        
        # Update previous error
        self.prev_distance_error = distance_error
        
        self.get_logger().debug(f"Path distance: {path_distance}, Error: {distance_error}, Control: {cmd.linear.x}")
        
        return cmd
    
    def calculate_path_distance_between_robots(self):
        """Calculate the distance along the path between leader and follower"""
        if not self.path_window or len(self.path_window) < 2:
            return self.distance(*self.follower_pos, *self.leader_pos)  # Fallback to Euclidean
        
        # Find the closest point on the path to follower's position
        follower_x, follower_y = self.follower_pos
        closest_idx = 0
        min_dist = float('inf')
        
        # First find the closest path point to follower
        for i, point in enumerate(self.leader_traj):
            px, py = point
            dist = self.distance(follower_x, follower_y, px, py)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Calculate the path length from follower's closest point to leader's current position
        path_distance = 0.0
        prev_x, prev_y = self.leader_traj[closest_idx]
        
        # Add path segments from closest point to leader's current position
        for i in range(closest_idx + 1, len(self.leader_traj)):
            curr_x, curr_y = self.leader_traj[i]
            path_distance += self.distance(prev_x, prev_y, curr_x, curr_y)
            prev_x, prev_y = curr_x, curr_y
        
        # Add final segment to leader's current position if needed
        leader_x, leader_y = self.leader_pos
        path_distance += self.distance(prev_x, prev_y, leader_x, leader_y)
        
        # For smoother control, blend with Euclidean distance when path is very short
        euclidean_distance = self.distance(follower_x, follower_y, leader_x, leader_y)
        if path_distance < 0.05:  # Very short path
            return euclidean_distance
        
        return path_distance
    
    @staticmethod
    def distance(x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def shutdown(self):
        self.get_logger().info('Shutting down. Stopping follower...')
        stop_cmd = Twist()
        self.cmd_pub_follower.publish(stop_cmd)
        self.destroy_node()

# def main():
#     rclpy.init()
#     node = PlatoonControl()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PlatoonControl()
    
    def signal_handler(sig, frame):
        node.shutdown()
        rclpy.shutdown()
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()
if __name__ == '__main__':
    main()