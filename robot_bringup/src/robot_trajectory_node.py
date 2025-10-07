#!/usr/bin/env python3

# import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class TrajectoryNode(Node):
    def __init__(self):

        super().__init__('robot_trajectory_node')
        
        # initialize publisher and subscriber objects for path
        self.path_pub = self.create_publisher(Path, 'robot_trajectory', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        self.path = Path()
        self.path.header.frame_id = 'world'   # the trajectory is calculated wrt. odom

    def odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.poses.append(pose)

        # Publish the path
        self.path_pub.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
