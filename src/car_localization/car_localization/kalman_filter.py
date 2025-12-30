#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from filterpy.kalman import KalmanFilter
import numpy as np

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter')

        self.odom_sub_ = self.create_subscription(Odometry, '/car_controller/odom', self.odom_callback, 10)
        self.imu_sub_ = self.create_subscription(Imu, '/imu/out', self.imu_callback, 10)

        self.kf_pub_ = self.create_publisher(Odometry, '/kalman_filter/odom', 10)

        self.imu_angular_z = 0.0
        self.is_first_odom = True
        self.last_angular_z = 0.0

        self.kalman_odom = Odometry()

        self.kf = KalmanFilter(dim_x=1, dim_z=2)

        self.kf.x = np.array([[0.0]])
        self.kf.F = np.array([[1.0]])
        self.kf.H = np.array([[1.0],  
                              [1.0]])
        self.kf.P = np.array([[1000.0]])
        self.kf.R = np.array([[0.5,  0.0], # IMU noise (lower = trust more)
                            [0.0,  5.0]])  # Odometry noise (higher = trust less)

        self.kf.Q = np.array([[0.4]])  # Process noise: var * dt = 4 * 0.1

    def imu_callback(self, msg):
        self.imu_angular_z = msg.angular_velocity.z

    def odom_callback(self, msg):
        self.kalman_odom = msg

        if self.is_first_odom:
            self.last_angular_z = msg.twist.twist.angular.z
            self.kf.x = np.array([[msg.twist.twist.angular.z]]) 
            self.is_first_odom = False
            return
        
        # Run Kalman Filter
        self.kf.predict() 
        self.kf.update(np.array([[self.imu_angular_z],
              [msg.twist.twist.angular.z]])) # update with true values

        self.last_angular_z = msg.twist.twist.angular.z

        self.kalman_odom.twist.twist.angular.z = self.kf.x[0, 0]
        self.kf_pub_.publish(self.kalman_odom)

        self.get_logger().info(
            f'Filtered angular.z: {self.kf.x[0, 0]:.3f}, '
            f'IMU: {self.imu_angular_z:.3f}, '
            f'Odom: {msg.twist.twist.angular.z:.3f}')

if __name__ == '__main__':
    rclpy.init()
    kalman_filter = KalmanFilterNode()
    rclpy.spin(kalman_filter)
    kalman_filter.destroy_node()
    rclpy.shutdown()
