import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

import sys
sys.path.append(f'/home/xiaobaige/anaconda3/envs/torch/lib/python3.10/site-packages/')
import torch

class ImageProcess(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.color_image_subscription = self.create_subscription(
            Image,
            '/color/image_raw',
            self.color_image_callback,
            10)
        # TODO: only subscribe once
        self.color_cm_info_subscription = self.create_subscription(
            CameraInfo,
            '/color/camera_info',
            self.color_cm_info_callback,
            10)
        self.depth_image_subscription = self.create_subscription(
            Image,
            '/depth/image_raw',
            self.depth_image_callback,
            10)
        # TODO: only subscribe once
        self.depth_cm_info_subscription = self.create_subscription(
            CameraInfo,
            '/depth/camera_info',
            self.depth_cm_info_callback,
            10)
    
    def color_image_callback(self, msg:Image):
        img = msg.data
        # img_tensor = torch.tensor(img)
        # do something here ...
        self.get_logger().info("get color image")

    def depth_image_callback(self, msg:Image):
        img = msg.data
        # img_tensor = torch.tensor(img)
        # do something here ...

    def color_cm_info_callback(self, msg:CameraInfo):
        # do something here ...
        # use `ros2 interface show sensor_msgs/msg/CameraInfo` in terminal 
        # to see the content of CameraInfo msg
        print(msg.k)

    def depth_cm_info_callback(self, msg:CameraInfo):
        # do something here ...
        # use `ros2 interface show sensor_msgs/msg/CameraInfo` in terminal 
        # to see the content of CameraInfo msg
        pass
        

def main(args=None):
    rclpy.init(args=args)
    img_process = ImageProcess()
    rclpy.spin(img_process)
    img_process.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()