#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math
from enum import Enum

class State(Enum):
    FREE = 0
    WARNING = 1
    DANGER = 2

class SafetyStop(Node):
    def __init__(self):
        super().__init__("safety_stop")

        self.declare_parameter("danger_distance", 0.2)
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("safety_stop_topic", "safety_stop") # twist_mux_locks.yaml
        
        self.danger_distance = self.get_parameter("danger_distance").get_parameter_value().double_value
        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        safety_stop_topic = self.get_parameter("safety_stop_topic").get_parameter_value().string_value

        self.scan_sub_ = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.safety_stop_pub_ = self.create_publisher(Bool, safety_stop_topic, 10) # block or not block twist_mux

        self.state = State.FREE

    def scan_callback(self, msg):
        self.state = State.FREE

        for range_value in msg.ranges:
            if not math.isinf(range_value) and range_value <= self.danger_distance: # short circuit
                self.state = State.DANGER   
                break
        
        is_safety_stop = Bool()
        if self.state == State.DANGER:
            is_safety_stop.data = True
            self.get_logger().info("LOCKED")
        elif self.state == State.FREE:
            is_safety_stop.data = False
            self.get_logger().info("FREE")

        self.safety_stop_pub_.publish(is_safety_stop)

if __name__ == "__main__":
    rclpy.init()
    node = SafetyStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()