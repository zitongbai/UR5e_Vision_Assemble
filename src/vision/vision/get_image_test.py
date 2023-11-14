import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

import sys
sys.path.append(f'/home/xiaobaige/anaconda3/envs/torch/lib/python3.10/site-packages/')
import torch

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/color/image_raw',
            self.listener_callback,
            10)
        self.subscription # prevent unused variable warning
    
    def listener_callback(self, msg:Image):
        # get image from msg
        img = msg.data
        # convert to tensor
        img_tensor = torch.tensor(img)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()