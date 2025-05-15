#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class ManualJointPublisher(Node):
    def __init__(self):
        super().__init__('manual_joint_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.joint1 = 0.0
        self.joint2 = 0.0

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2']
        msg.position = [self.joint1, self.joint2]
        self.publisher_.publish(msg)

    def set_angles_deg(self, deg1, deg2):
        self.joint1 = math.radians(deg1)
        self.joint2 = math.radians(deg2)
        self.get_logger().info(f"Updated angles to: joint1={deg1}°, joint2={deg2}°")

def main():
    rclpy.init()
    node = ManualJointPublisher()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            inp = input("Enter joint1 and joint2 angles (degrees, e.g. 30 45): ")
            try:
                j1, j2 = map(float, inp.strip().split())
                node.set_angles_deg(j1, j2)
            except ValueError:
                print("❌ Invalid input. Please enter two numbers like: 30 45")
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
