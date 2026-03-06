import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import base64
import numpy as np
from cv_bridge import CvBridge
from .utils import get_bboxes

class CarPoseNode(Node):
    def __init__(self):
        super().__init__('car_pose_node')
        self.get_logger().info('Car Pose Node has been started.')
        self.bridge = CvBridge()

        self.camera_sub = self.create_subscription(
            Image,
            '/camera',
            self.camera_callback,
            10
        )

        self.cv_pub = self.create_publisher(Image, '/estimator/image', 10)

    def _cv_to_base64(self, cv_image: np.ndarray) -> str:
        ok, buffer = cv2.imencode('.jpg', cv_image)
        if not ok:
            raise RuntimeError('cv2.imencode failed while converting image to base64')
        return base64.b64encode(buffer).decode('utf-8')

    def camera_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        image_base64 = self._cv_to_base64(cv_image)
        bbox_info = get_bboxes(image_base64=image_base64)

        annotated_base64 = bbox_info["img"]
        annotated_bytes = base64.b64decode(annotated_base64)
        annotated_array = np.frombuffer(annotated_bytes, dtype=np.uint8)
        annotated_cv = cv2.imdecode(annotated_array, cv2.IMREAD_COLOR)
        if annotated_cv is None:
            self.get_logger().error('Failed to decode annotated image from base64.')
            return

        out_msg = self.bridge.cv2_to_imgmsg(annotated_cv, encoding='bgr8')
        out_msg.header = msg.header
        self.cv_pub.publish(out_msg)
