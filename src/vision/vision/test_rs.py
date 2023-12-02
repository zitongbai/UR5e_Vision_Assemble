import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
import numpy as np
import matplotlib.pyplot as plt

class TestRs(Node):
    def __init__(self):
        super().__init__('test_rs')
        self.subscription = self.create_subscription(
            Image,
            '/depth_registered/image_rect',
            self.depth_callback,
            10)
    
    def depth_callback(self, msg):
        bridge = CvBridge()
        depth = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # draw heat map
        plt.imshow(depth, cmap='jet')
        plt.colorbar()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    test_rs = TestRs()
    rclpy.spin(test_rs)
    test_rs.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()