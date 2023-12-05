import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import CompressedImage
from .wait_for_message import wait_for_message

import cv2
import numpy as np
import matplotlib.pyplot as plt

class DepthImageSync(Node):
    def __init__(self):
        super().__init__('test_sync')
        self.get_logger().info("test_sync node started")
        # self.color_compressed_sub = self.create_subscription(CompressedImage, '/color/image_raw/compressed', self.color_compressed_callback, 10)
        self.depth_compressed_sub = self.create_subscription(CompressedImage, '/depth_registered/image_rect/compressedDepth', self.depth_compressed_callback, 10)

    def color_compressed_callback(self, msg:CompressedImage):
        #### direct conversion to CV2 ####
        # REF: http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
        
        np_arr = np.asarray(bytearray(msg.data), dtype=np.uint8)
        img_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        cv2.namedWindow("Image window", cv2.WINDOW_NORMAL|cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow("Image window", img_np.shape[1], img_np.shape[0])
        cv2.imshow("Image window", img_np)
        cv2.waitKey(1)

    def depth_compressed_callback(self, msg:CompressedImage):
        #### direct conversion to CV2 ####
        # REF: http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
        # REF: https://stackoverflow.com/questions/41051998/ros-compresseddepth-to-numpy-or-cv2
        # REF: https://robotics.stackexchange.com/questions/77192/decode-compresseddepth-message-from-compressed-depth-image-transport
        
        depth_fmt, compr_type = msg.format.split(';')
        depth_fmt = depth_fmt.strip()
        compr_type = compr_type.strip()
        if compr_type != "compressedDepth":
            raise Exception("Compression type is not 'compressedDepth'."
                            "You probably subscribed to the wrong topic.")
        
        # remove header from raw data
        depth_header_size = 6
        raw_data = msg.data[depth_header_size:]

        depth = cv2.imdecode(np.asarray(raw_data, dtype=np.uint16), cv2.IMREAD_UNCHANGED)

        cv2.namedWindow("Depth window", cv2.WINDOW_NORMAL|cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow("Depth window", depth.shape[1], depth.shape[0])
        cv2.imshow("Depth window", depth)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DepthImageSync()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()