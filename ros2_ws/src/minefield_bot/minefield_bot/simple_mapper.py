import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Int32
import numpy as np
import math

class MineMapper(Node):
    def __init__(self):
        super().__init__('mine_mapper')
        
        self.map_pub = self.create_publisher(OccupancyGrid, 'minefield_map', 10)
        self.create_subscription(Int32, 'metal_detector_raw', self.sensor_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        self.current_x = 0.0
        self.current_y = 0.0
        
        # Grid Setup
        self.width = 100
        self.height = 100
        self.resolution = 0.05 # 5cm per cell
        self.grid = np.zeros((self.width, self.height)) # 0=Free
        
        self.create_timer(1.0, self.publish_map)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def sensor_callback(self, msg):
        # Only mark map if we have metal detection
        if msg.data > 400: # Threshold depends on your sensor
            gx = int(self.current_x / self.resolution) + (self.width // 2)
            gy = int(self.current_y / self.resolution) + (self.height // 2)
            
            if 0 <= gx < self.width and 0 <= gy < self.height:
                # Add probability (Simple Bayesian update)
                self.grid[gy, gx] = min(100, self.grid[gy, gx] + 20)

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom" # Map is fixed to Odom frame for this simple version
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = -(self.width * self.resolution) / 2.0
        msg.info.origin.position.y = -(self.height * self.resolution) / 2.0
        
        # Flatten and cast
        msg.data = self.grid.flatten().astype(int).tolist()
        self.map_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MineMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
