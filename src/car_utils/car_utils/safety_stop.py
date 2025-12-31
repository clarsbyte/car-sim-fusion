#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math
from enum import Enum
from visualization_msgs.msg import Marker, MarkerArray

class State(Enum):
    FREE = 0
    WARNING = 1
    DANGER = 2

class SafetyStop(Node):
    def __init__(self):
        super().__init__("safety_stop")

        self.declare_parameter("danger_distance", 0.2)
        self.declare_parameter("warning_distance", 0.5)
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("safety_stop_topic", "safety_stop") # twist_mux_locks.yaml
        
        self.danger_distance = self.get_parameter("danger_distance").get_parameter_value().double_value
        self.warning_distance = self.get_parameter("warning_distance").get_parameter_value().double_value
        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        safety_stop_topic = self.get_parameter("safety_stop_topic").get_parameter_value().string_value

        self.scan_sub_ = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.safety_stop_pub_ = self.create_publisher(Bool, safety_stop_topic, 10) # block or not block twist_mux
        self.zones_pub_ = self.create_publisher(MarkerArray, "zones", 10)

        self.state = State.FREE
        self.is_first_msg = True

        self.zones = MarkerArray()

        warning_zone = Marker()
        warning_zone.id = 0
        warning_zone.action = Marker.ADD
        warning_zone.type = Marker.CYLINDER
        warning_zone.scale.z = 0.001
        warning_zone.scale.x = self.warning_distance * 2
        warning_zone.scale.y = self.warning_distance * 2
        warning_zone.color.r = 1.0
        warning_zone.color.g = 0.984
        warning_zone.color.b = 0.0
        warning_zone.color.a = 0.5
        
        danger_zone = Marker()
        danger_zone.id = 1
        danger_zone.action = Marker.ADD
        danger_zone.type = Marker.CYLINDER
        danger_zone.scale.z = 0.001
        danger_zone.scale.x = self.danger_distance * 2
        danger_zone.scale.y = self.danger_distance * 2
        danger_zone.color.r = 1.0
        danger_zone.color.g = 0.0
        danger_zone.color.b = 0.0
        danger_zone.color.a = 0.5
        danger_zone.pose.position.z = 0.01 # so no overlap
        self.zones.markers = [warning_zone, danger_zone]

    def scan_callback(self, msg):
        self.state = State.FREE

        for range_value in msg.ranges:
            if not math.isinf(range_value) and range_value <= self.danger_distance: # short circuit
                self.state = State.DANGER   
                break
        
        is_safety_stop = Bool()
        if self.state == State.WARNING:
            is_safety_stop.data = False
            self.get_logger().info("WARNING")
            self.zones.markers[0].color.a = 1.0
            self.zones.markers[1].color.a = 0.3
        if self.state == State.DANGER:
            is_safety_stop.data = True
            self.zones.markers[0].color.a = 1.0
            self.zones.markers[1].color.a = 1.0
            self.get_logger().info("LOCKED")
        elif self.state == State.FREE:
            is_safety_stop.data = False
            self.zones.markers[0].color.a = 0.5
            self.zones.markers[1].color.a = 0.5
            self.get_logger().info("FREE")

        self.safety_stop_pub_.publish(is_safety_stop)
        
        if self.is_first_msg:
            self.is_first_msg = False
            for zone in self.zones.markers:
                zone.header.frame_id = msg.header.frame_id
        
        self.zones_pub_.publish(self.zones)

if __name__ == "__main__":
    rclpy.init()
    node = SafetyStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()