#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray
import numpy as np

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('wheel_separation', 0.17)

        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value

        self.get_logger().info(f'Wheel Radius: {self.wheel_radius}, Wheel Separation: {self.wheel_separation}')

        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.vel_sub_ = self.create_subscription(TwistStamped, "car_controller/cmd_vel", self.vel_callback, 10)

        self.speed_conversion_ = np.array([[self.wheel_radius / 2, self.wheel_radius / 2],
                                           [self.wheel_radius / self.wheel_separation, -self.wheel_radius / self.wheel_separation]])

    def vel_callback(self, msg: TwistStamped):
        robot_speed = np.array([[msg.twist.linear.x], # linear velocity of robot along x axis
                                [msg.twist.angular.z]]) # no y to respect differential drive

        wheel_speed = np.linalg.inv(self.speed_conversion_) @ robot_speed
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1,0], wheel_speed[0,0]] # left wheel, right wheel
        self.wheel_cmd_pub_.publish(wheel_speed_msg)

if __name__ == '__main__':
    rclpy.init()
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()