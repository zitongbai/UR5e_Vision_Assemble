import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/depth_registered/image_rect',
            self.listener_callback,
            10)
        self.subscription # prevent unused variable warning
        
    
    def listener_callback(self, msg:Image):
        self.get_logger().info('get image')
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        plt.hist(cv_img.ravel(), 100, [0, 4],)
        plt.savefig('hist.png')
        

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin_once(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()