#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException

class Pose:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

def coordinatesToPose(x, y, map_info: MapMetaData):
    pose = Pose()
    pose.x = int((x - map_info.origin.position.x) / map_info.resolution)
    pose.y = int((y - map_info.origin.position.y) / map_info.resolution)
    return pose

def poseOnMap(pose: Pose, map_info: MapMetaData):
    return pose.x >= 0 and pose.x < map_info.width and pose.y >= 0 and pose.y < map_info.height

def poseToCell(pose: Pose, map_info: MapMetaData):
    return map_info.width * pose.y + pose.x

class MappingWithKnownPoses(Node):
    def __init__(self, name):
        super().__init__(name)

        # Occupancy grid size
        self.declare_parameter('width', 50.0) # width of map
        self.declare_parameter('height', 50.0) # height of map
        self.declare_parameter('resolution', 0.1) # resolution of map

        width = self.get_parameter('width').get_parameter_value().double_value
        height = self.get_parameter('height').get_parameter_value().double_value
        resolution = self.get_parameter('resolution').get_parameter_value().double_value

        self.map_ = OccupancyGrid()
        self.map_.info.resolution = resolution
        # below in number of cells
        self.map_.info.width = int(width / resolution) 
        self.map_.info.height = int(height / resolution)
        self.map_.info.origin.position.x = float(round(-width / 2.0))
        self.map_.info.origin.position.y = float(round(-height / 2.0))
        self.map_.header.frame_id = "odom"
        self.map_.data = [-1] * (self.map_.info.width * self.map_.info.height) # the cells in the grid are unknown initially

        self.map_publisher_ = self.create_publisher(OccupancyGrid, 'map', 1)
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        self.timer = self.create_timer(1.0, self.timer_callback) # repeatedly do timer_callback every second

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)  

    def scan_callback(self, scan_msg):  
        # read current position of robot
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_.header.frame_id, # transform to this frame
                scan_msg.header.frame_id, # from this frame (b/w them)
                rclpy.time.Time()
            )
        except LookupException as e:
            self.get_logger().error(f"Transform lookup failed: {e}")
            return

        # marking robot's posn

        # converting to a point in occupancy grid

        robot_position = coordinatesToPose(t.transform.translation.x, t.transform.translation.y, self.map_.info) 
        if not poseOnMap(robot_position, self.map_.info): # check whether within map boundary
            self.get_logger().warn("Robot position is out of map bounds.")
            return
        
        robot_cell = poseToCell(robot_position, self.map_.info) # convert to cell
        self.map_.data[robot_cell] = 100 # mark robot's cell as occupied

    def timer_callback(self):
        self.map_.header.stamp = self.get_clock().now().to_msg() 
        self.map_publisher_.publish(self.map_)

if __name__ == '__main__':
    rclpy.init()
    node = MappingWithKnownPoses('mapping_with_known_poses')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()