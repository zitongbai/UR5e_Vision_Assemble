# ros2 import
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time
import numpy as np
from .wait_for_message import wait_for_message

# address path 
import os
import sys
from pathlib import Path
# add conda env path
sys.path.append(str(Path.home()) + '/miniforge3/envs/yolo/lib/python3.10/site-packages')

# yolov5 import
FILE = Path(__file__).resolve()
ROOT = os.path.join(FILE.parents[0], 'yolov5')  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

import torch
from ultralytics.utils.plotting import Annotator, colors
from .yolov5.models.common import DetectMultiBackend
from .yolov5.utils.torch_utils import select_device
from .yolov5.utils.general import check_img_size, check_imshow, non_max_suppression, scale_boxes
from .yolov5.utils.augmentations import letterbox




class ObjDetect(Node):
    def __init__(self):
        super().__init__('obj_detect')
        self.get_logger().info('obj_detect node started')
        self.bridge = CvBridge()
        # ROS2 parameters declaration
        self.declare_parameter("weights_path", str(ROOT / 'yolov5s.pt'), ParameterDescriptor(
            name="weights_path", description="path to weights file"))
        
        self.declare_parameter("data_path", str(ROOT / 'data/coco128.yaml'), ParameterDescriptor(
            name="data_path", description="path to dataset.yaml"))

        self.declare_parameter("image_topic", "/color/image_raw", ParameterDescriptor(
            name="image_topic", description="image topic"))
        
        self.declare_parameter("image_info_topic", "/color/camera_info", ParameterDescriptor(
            name="camera_info_topic", description="camera info topic"))
        
        self.declare_parameter("depth_topic", "/depth_registered/image_rect", ParameterDescriptor(
            name="depth_topic", description="depth image topic"))
        
        self.declare_parameter("depth_info_topic", "/depth_registered/camera_info", ParameterDescriptor(
            name="depth_info_topic", description="depth camera info topic"))
        
        self.declare_parameter("detection_freq", 30, ParameterDescriptor(
            name="detection_freq", description="detection frequency [Hz]"))
        
        self.declare_parameter("view_image", True, ParameterDescriptor(
            name="view_image", description="whether to show the detect result in a window"))

        self.declare_parameter("publish_result", True, ParameterDescriptor(
            name="publish_result", description="whether to publish the detect result"))
        
        # set parameter use_sim_time to true
        use_sim_time = rclpy.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            True
        )
        self.set_parameters([use_sim_time])   

        # ROS2 camera info subscriber
        image_info_topic = self.get_parameter("image_info_topic").value
        depth_info_topic = self.get_parameter("depth_info_topic").value
        success, self.image_info = wait_for_message(CameraInfo, self, image_info_topic, time_to_wait=10.0)
        if not success:
            self.get_logger().error(f"cannot get camera info from {image_info_topic}")
            exit()
        success, self.depth_info = wait_for_message(CameraInfo, self, depth_info_topic, time_to_wait=10.0)
        if not success:
            self.get_logger().error(f"cannot get depth camera info from {depth_info_topic}")
            exit()
        self.imgsz = (self.image_info.height, self.image_info.width)
        self.depth_instrinsic = np.array(self.depth_info.k, dtype=np.float32).reshape(3, 3)
        self.depth_instrinsic_inv = np.linalg.inv(self.depth_instrinsic)

        # ROS2 detection result publisher
        self.publish_result = self.get_parameter("publish_result").value
        self.detection_pub = self.create_publisher(Detection2DArray, 'detection', 10)
        self.detection_result = Detection2DArray()

        # load yolov5 model
        self.half = False # use FP16 half-precision inference
        device = select_device('0')
        weights = self.get_parameter("weights_path").value
        data = self.get_parameter("data_path").value
        self.get_logger().info(f"loading model with weights in {weights}")
        self.model = DetectMultiBackend(weights, device=device, dnn=False, data=data, fp16=self.half)
        self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt
        self.imgsz = check_img_size(self.imgsz, s=self.stride)
        self.get_logger().info(f'yolov5 model warmup')
        self.model.warmup(imgsz=(1, 3, *self.imgsz)) # BCHW
        self.view_img = check_imshow(warn=True) and self.get_parameter("view_image").value # check if we can view image

        # ROS2 image, depth subscriber
        # the callback function depends on yolov5 model, thus init after model
        image_topic = self.get_parameter("image_topic").value
        depth_topic = self.get_parameter("depth_topic").value

        self.image = Image()
        self.depth = Image()
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        
        # ROS2 timer
        self.detection_freq = self.get_parameter("detection_freq").value
        self.timer = self.create_timer(1.0 / self.detection_freq, self.timer_callback)


    def depth_callback(self, msg:Image):
        self.depth = msg
    
    def image_callback(self, msg:Image):
        self.image = msg

    def timer_callback(self):
        """
        Callback function for timer
        Process the image and depth data, and publish the detection result
        in frequency of detection_freq
        """

        # check if we have received up to date image and depth
        tol_sec = 0.5
        image_stamp = self.image.header.stamp.sec + self.image.header.stamp.nanosec * 1e-9
        depth_stamp = self.depth.header.stamp.sec + self.depth.header.stamp.nanosec * 1e-9
        now_stamp = self.get_clock().now().nanoseconds * 1e-9
        if abs(image_stamp - now_stamp) > tol_sec or abs(depth_stamp - now_stamp) > tol_sec:
            self.get_logger().warn(f"image stamp = {image_stamp}, depth stamp = {depth_stamp}, now stamp = {now_stamp}")
            if abs(image_stamp - now_stamp) > tol_sec:
                self.get_logger().warn(f"image stamp is not up to date")
            if abs(depth_stamp - now_stamp) > tol_sec:
                self.get_logger().warn(f"depth stamp is not up to date")
            return
        
        # image preprocessing
        dep = self.bridge.imgmsg_to_cv2(self.depth, desired_encoding= 'passthrough')
        img = self.bridge.imgmsg_to_cv2(self.image, desired_encoding= 'bgr8') # HWC
        img0 = img.copy() # for visualization
        # Padded resize
        img = letterbox(img, new_shape=self.imgsz, stride=self.stride, auto=self.pt)[0]
        img = img.transpose(2, 0, 1) # HWC to CHW
        img = np.ascontiguousarray(img) # contiguous array for memory efficiency
        img = torch.from_numpy(img).to(self.model.device) # convert to torch gpu
        img = img.half() if self.half else img.float()
        img /= 255.0 # 0 - 255 to 0.0 - 1.0
        if len(img.shape) == 3:
            img = img[None] # expand for batch dim
        
        # inference
        augment = False # augmented inference
        pred = self.model(img, augment=augment, visualize=False)

        # NMS
        conf_thres = 0.25
        iou_thres = 0.45
        classes = None # optional filter by class
        agnostic_nms = False # class-agnostic NMS
        max_det = 1000 # maximum detections per image
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)

        # process predictions
        det = pred[0] # we only has one image
        annotator = Annotator(img0, line_width=3, example=str(self.names))
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], img0.shape).round()

            # prepare detection result msg and annote results
            for *xyxy, conf, cls in reversed(det):
                # prepare detection result msg
                detection = Detection2D()
                detection.id = self.names[int(cls)]
                x1, y1, x2, y2 = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])
                detection.bbox.center.position.x = (x1 + x2) / 2.0
                detection.bbox.center.position.y = (y1 + y2) / 2.0
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)

                # add hypothesis to detection result msg
                obj_hypothesis = ObjectHypothesisWithPose()
                obj_hypothesis.hypothesis.class_id = self.names[int(cls)]
                obj_hypothesis.hypothesis.score = float(conf)
                Z = dep[int(detection.bbox.center.position.y), int(detection.bbox.center.position.x)]
                Z = Z * 1e-3 # mm to m
                uv1 = np.array([detection.bbox.center.position.x, detection.bbox.center.position.y, 1.0])
                XZ_YZ_1 = np.dot(self.depth_instrinsic_inv, uv1)
                XYZ = np.array([XZ_YZ_1[0] * Z, XZ_YZ_1[1] * Z, Z])
                obj_hypothesis.pose.pose.position.x = XYZ[0]
                obj_hypothesis.pose.pose.position.y = XYZ[1]
                obj_hypothesis.pose.pose.position.z = XYZ[2]

                detection.results.append(obj_hypothesis)
                self.detection_result.detections.append(detection)

                # annotate results
                if self.view_img:
                    c = int(cls)
                    label = f'{self.names[c]} {conf:.2f}'
                    annotator.box_label(xyxy, label, color=colors(c, True))
                    # use cv2 draw a red point labeled with XYZ[2] in img0
                    cv2.circle(img0, (int(detection.bbox.center.position.x), int(detection.bbox.center.position.y)), 2, (0, 0, 255), -1)
                    cv2.putText(img0, f'{XYZ[2]:.2f}', (int(detection.bbox.center.position.x), int(detection.bbox.center.position.y)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA)
        
        if self.view_img:
            img0 = annotator.result()
            cv2.namedWindow('obj_detect', cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
            cv2.resizeWindow('obj_detect', img0.shape[1], img0.shape[0])
            cv2.imshow('obj_detect', img0)
            cv2.waitKey(1)
        
        if self.publish_result:
            self.detection_result.header.stamp = self.get_clock().now().to_msg()
            self.detection_pub.publish(self.detection_result)


def main(args=None):
    rclpy.init()
    obj_det_node = ObjDetect()
    rclpy.spin(obj_det_node)
    obj_det_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


