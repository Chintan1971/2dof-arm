#!/usr/bin/env python3

# move_object.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from ros_gz_interfaces.srv import SetEntityPose 
import random
import math

class ObjectMoverClient(Node):
    def __init__(self):
        super().__init__('object_mover_client')
        self.cli = self.create_client(SetEntityPose, '/world/empty/set_pose')
        while not self.cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('Waiting for /set_pose service...')

        self.t = 0.0
        self.timer = self.create_timer(2, self.send_req)

    def send_req(self):
        self.t += 0.02
        req = SetEntityPose.Request()
        req.entity.name = 'movable_object'
        req.entity.type = 2
        req.pose = Pose()
        req.pose.position.x = random.uniform(1, 3)  
        req.pose.position.y = random.uniform(0, 1)  
        req.pose.position.z = 0.5  # Fixed height for simplicity
        req.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        # req.pose.position = Point(x=1.0 + 0.5 * math.sin(self.t),
        #                           y=0.5 * math.cos(self.t),
        #                           z=0.6 + 0.2 * math.sin(self.t * 2)
        #                           )
        # req.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.get_logger().info('Sending request to move object')
        self.cli.call_async(req)
        # future.add_done_callback(self.handle_response)

    # def handle_response(self, future):
        # try:
            # response = future.result()
            # if response.success:
                # self.get_logger().info('Object moved successfully')
            # else:
                # self.get_logger().warn('Move failed')
        # except Exception as e:
            # self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectMoverClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
