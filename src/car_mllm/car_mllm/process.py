from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import base64
from cv_bridge import CvBridge
import rclpy
from .mllm import process_image_with_text
from geometry_msgs.msg import Twist


class VisionAction(Node):
    def __init__(self):
        super().__init__("vision_action")

        self.declare_parameter("target", "Go to the fridge")
        self.target = self.get_parameter("target").get_parameter_value().string_value

        self.cam_sub_ = self.create_subscription(Image, "camera", self.cam_callback, 10)
        self.action_pub_ = self.create_publisher(Twist, "nav/cmd_vel", 10) # mock for now
        self.bridge = CvBridge()

    def cam_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.get_logger().info('Received image for processing.')
        
        _, buffer = cv2.imencode('.jpg', cv_image)
        
        image_base64 = base64.b64encode(buffer).decode('utf-8')

        res = process_image_with_text(image_base64, self.target)
        self.get_logger().info(f'Vision Action Result: {res}')
        action_msg = Twist()
        action_msg.linear.x = res.velocity_x
        action_msg.angular.z = res.angular_z

        self.action_pub_.publish(action_msg)

def main(args=None):
    rclpy.init(args=args)
    vision_action = VisionAction()
    rclpy.spin(vision_action)
    vision_action.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()