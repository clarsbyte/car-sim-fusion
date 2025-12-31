# Twist Mux Nav
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist

class ActionMove(Node):
    def __init__(self):
        super().__init__("action_move")

        self.twist_pub_ = self.create_publisher(Twist, "nav/cmd_vel", 10)
        self.timer = self.create_timer(0.1, self.publish_cmd)  # 10 Hz

    def publish_cmd(self):
        twist = Twist()
        twist.linear.x = 0.5  # forward speed (m/s)
        twist.angular.z = 0.0 
        self.twist_pub_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ActionMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
