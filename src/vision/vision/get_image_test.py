import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
import time
import os

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/color/image_raw',
            self.listener_callback,
            10)
        self.subscription # prevent unused variable warning
        
    
    def listener_callback(self, msg:Image):
        self.get_logger().info('get image')
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_path = '/home/xiaobaige/Projects/gazebo_dataset/images/'
        img_name = time.strftime("%Y%m%d-%H%M%S") + '.jpg'
        cv2.imwrite(os.path.join(img_path, img_name), cv_img)

        # plt.hist(cv_img.ravel(), 100, [0, 4],)
        # plt.savefig('hist.png')
        

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin_once(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()