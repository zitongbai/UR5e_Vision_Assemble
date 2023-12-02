import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

from .wait_for_message import wait_for_message

# we use ComposableNodeContainer to launch `convert_metric_node` 
# and `register_node`, ref: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Composition.html
# test the sync of depth image and color image

class DepthImageSync(Node):
    def __init__(self):
        super().__init__('depth_image_sync')

        self.color_sub = self.create_subscription(Image, '/color/image_raw', self.color_callback, 10)
        self.depth_converted_sub = self.create_subscription(Image, '/depth/converted_image', self.depth_converted_callback, 10)
        self.depth_registered_sub = self.create_subscription(Image, '/depth_registered/image_rect', self.depth_registered_callback, 10)

        self.color_stamp = None 
        self.depth_converted_stamp = None
        self.depth_registered_stamp = None

    def color_callback(self, msg: Image):
        self.color_stamp = msg.header.stamp
        self.check_sync()
    
    def depth_converted_callback(self, msg: Image):
        self.depth_converted_stamp = msg.header.stamp
        self.check_sync()

    def depth_registered_callback(self, msg: Image):
        self.depth_registered_stamp = msg.header.stamp
        self.check_sync()
    
    def check_sync(self):
        if self.color_stamp is None or self.depth_converted_stamp is None or self.depth_registered_stamp is None:
            return
        tol = 0.01
        if abs(self.color_stamp.sec - self.depth_converted_stamp.sec) < tol and abs(self.color_stamp.sec - self.depth_registered_stamp.sec) < tol:
            self.get_logger().info('synced')
        else:
            self.get_logger().info('not synced')
        
        self.color_stamp = None 
        self.depth_converted_stamp = None
        self.depth_registered_stamp = None

def main(args=None):
    rclpy.init(args=args)
    node = DepthImageSync()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()