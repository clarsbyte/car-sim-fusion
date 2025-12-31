# Twist Mux Nav
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist

class ActionMove(Node):
    def __init__(self):
        super().__init__("action_move")

        self.twist_pub_ = self.create_publisher(Twist, "nav/cmd_vel", 10)
        self.mllm_sub_ = self.create_subscription(Twist, "vision_action/output", self.mllm_callback, 10)

    def mllm_callback(self, msg):
        twist = Twist()
        twist.linear.x = msg.linear.x
        twist.angular.z = msg.angular.z
        self.twist_pub_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ActionMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
