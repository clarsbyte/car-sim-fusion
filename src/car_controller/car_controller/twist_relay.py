#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from rosgraph_msgs.msg import Clock

class TwistRelayNode(Node):
    def __init__(self):
        super().__init__("twist_relay")
        self.sim_time = None

        self.clock_sub = self.create_subscription(
            Clock,
            "/clock",
            self.clock_callback,
            10
        )
        self.sub = self.create_subscription(
            Twist,
            "/twist_mux/cmd_vel",
            self.twist_callback,
            10
        )
        self.pub = self.create_publisher(
            TwistStamped,
            "/car_controller/cmd_vel",
            10
        )
        self.get_logger().info("Twist relay started")

    def clock_callback(self, msg):
        self.sim_time = msg.clock

    def twist_callback(self, msg):
        twist_stamped = TwistStamped()
        if self.sim_time is not None:
            twist_stamped.header.stamp = self.sim_time
        else:
            # fallback to wall time if no sim clock
            now = self.get_clock().now().to_msg()
            twist_stamped.header.stamp = now
        twist_stamped.header.frame_id = "base_link"
        twist_stamped.twist = msg
        self.pub.publish(twist_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = TwistRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
