import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import CompressedImage
from .wait_for_message import wait_for_message

import cv2
from cv_bridge import CvBridge
import numpy as np
import matplotlib.pyplot as plt

class DepthImageSync(Node):
    def __init__(self):
        super().__init__('test_sync')
        self.get_logger().info("test_sync node started")
        self.bridge = CvBridge()
        # self.color_compressed_sub = self.create_subscription(CompressedImage, '/color/image_raw/compressed', self.color_compressed_callback, 10)
        # self.depth_compressed_sub = self.create_subscription(CompressedImage, 
        #                                                     #  '/depth_registered/image_rect/compressedDepth', 
        #                                                     '/depth/image_raw/compressedDepth',
        #                                                      self.depth_compressed_callback, 
        #                                                      qos_profile_sensor_data)

        self.depth_sub = self.create_subscription(Image, 
                                                  '/depth/image_raw',
                                                  self.depth_callback, 
                                                  10
                                                  )

    def color_compressed_callback(self, msg:CompressedImage):
        #### direct conversion to CV2 ####
        # REF: http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
        
        img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # np_arr = np.asarray(bytearray(msg.data), dtype=np.uint8)
        # img_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        

        cv2.namedWindow("Image window", cv2.WINDOW_NORMAL|cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow("Image window", img.shape[1], img.shape[0])
        cv2.imshow("Image window", img)
        cv2.waitKey(1)

    def depth_compressed_callback(self, msg:CompressedImage):
        #### direct conversion to CV2 ####
        # REF: http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
        # REF: https://stackoverflow.com/questions/41051998/ros-compresseddepth-to-numpy-or-cv2
        # REF: https://robotics.stackexchange.com/questions/77192/decode-compresseddepth-message-from-compressed-depth-image-transport
        pass        
        # depth_fmt, compr_type = msg.format.split(';')
        # depth_fmt = depth_fmt.strip()
        # compr_type = compr_type.strip()
        # if compr_type != "compressedDepth":
        #     raise Exception("Compression type is not 'compressedDepth'."
        #                     "You probably subscribed to the wrong topic.")
        
        # # remove header from raw data
        # str_msg = msg.data
        # buf = np.ndarray(shape=(1, len(str_msg)), dtype=np.uint8, buffer=str_msg)

        # depth_header_size = 12
        # buf = buf[0, depth_header_size:]

        # depth = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)

        # # draw heat map of depth using matplotlib
        # plt.imshow(depth, cmap='hot', interpolation='nearest')
        # plt.show()

    def depth_callback(self, msg:Image):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = DepthImageSync()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()