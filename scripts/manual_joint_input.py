#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # Change this
import math

class ManualJointPublisher(Node):
    def __init__(self):
        super().__init__('manual_joint_publisher')
        # Change topic name and message type
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        # self.timer = self.create_timer(0.1, self.timer_callback) # Timer might not be needed if publishing on input
        self.joint1_rad = 0.0 # Store in radians
        self.joint2_rad = 0.0 # Store in radians

    # This timer_callback will be replaced by a publish_command method
    # def timer_callback(self):
    #     msg = JointState()
    #     msg.header.stamp = self.get_clock().now().to_msg()
    #     msg.name = ['joint1', 'joint2']
    #     msg.position = [self.joint1, self.joint2]
    #     self.publisher_.publish(msg)

    # New method to publish JointTrajectory
    def publish_command(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['joint1', 'joint2']

        point = JointTrajectoryPoint()
        point.positions = [self.joint1_rad, self.joint2_rad]
        point.time_from_start.sec = 0 # Reach target quickly
        point.time_from_start.nanosec = 100000000 # 0.1 seconds

        msg.points = [point]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing command: joint1={math.degrees(self.joint1_rad)}°, joint2={math.degrees(self.joint2_rad)}°")

    def set_angles_deg(self, deg1, deg2):
        self.joint1_rad = math.radians(deg1)
        self.joint2_rad = math.radians(deg2)
        self.get_logger().info(f"Target angles set to: joint1={deg1}°, joint2={deg2}°")
        self.publish_command() # Call the new publish method

def main():
    rclpy.init()
    node = ManualJointPublisher()

    try:
        while rclpy.ok():
            inp = input("Enter joint1 and joint2 angles (degrees, e.g. 30 45): ")
            try:
                j1, j2 = map(float, inp.strip().split())
                node.set_angles_deg(j1, j2) # This triggers the publish
            except ValueError:
                node.get_logger().warn("Invalid input. Please enter two numbers like: 30 45")
            # Add a small spin to allow ROS2 communications to process
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Manual joint publisher shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
