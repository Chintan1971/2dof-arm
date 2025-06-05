#!/usr/bin/env python3

# move_object.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
# from ros_gz_interfaces.msg import Pose_V as GzPose
from ros_gz_interfaces.msg import EntityFactory
# from ros_gz_interfaces.srv import SetEntityPose 
from std_msgs.msg import String
import math
import time

class ObjectMover(Node):
    def __init__(self):
        super().__init__('object_mover')
        self.publisher_ = self.create_publisher(EntityFactory, '/world/default/entity/set_pose', 10)
        self.timer = self.create_timer(0.05, self.move_object)
        self.t = 0.0

    def move_object(self):
        self.t += 0.05
        pose = Pose()
        pose.name = 'my_cube'
        pose.position.x = 1.0 + 0.5 * math.sin(self.t)
        pose.position.y = 0.5 * math.cos(self.t)
        pose.position.z = 0.5
        pose.orientation.w = 1.0

        msg = EntityFactory()
        msg.name = 'my_cube'
        msg.pose = Pose()

        self.publisher_.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = ObjectMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
