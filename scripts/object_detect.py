import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np  

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info('Object Detector Node has been started.')

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        #red color mask
        lower_red1  = np.array([100, 150, 50])
        upper_red1  = np.array([140, 255, 255])
        # lower_red2  = np.array([170, 120, 70])
        # upper_red2  = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv,lower_red1,upper_red1)
        # mask2 = cv2.inRange(hsv,lower_red2,upper_red2)

        # mask = mask1 | mask2
        
        contours, _ = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M['m00'] != 0:
                cX = int(M['m10'] / M['m00'])
                cY = int(M['m01'] / M['m00'])
                self.get_logger().info(f'Object detected at ({cX}, {cY})')

                cv2.circle(frame, (cX, cY), 5, (0, 255, 0), -1)

            else:
                self.get_logger().warn('No object detected')
        else:
            self.get_logger().warn('No contours found')
            
        cv2.imshow('Object Detection', frame)
        cv2.waitKey(1)
    
def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
            