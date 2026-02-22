#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from PIL import Image
import numpy as np
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

class GroundTexturePublisher(Node):
    def __init__(self):
        super().__init__('ground_texture_publisher')
        
        # Declare and evaluate parameters
        paramList = [
            ('image_path', '/path/to/ground_texture.png'),
            ('resolution', 0.1),  # meters per pixel
            ('origin_x', -50.0),  # Map origin (meters)
            ('origin_y', -50.0),
        ]
        for name, defaultValue in paramList:
            self.declare_parameter(name, defaultValue)
            setattr(self, name, self.get_parameter(name).value)
        
        # Load image
        try:
            img = Image.open(self.image_path).convert('L')  # Grayscale
            self.get_logger().info(f'Loaded image: {img.size}')
        except Exception as e:
            self.get_logger().error(f'Failed to load image: {e}')
            # Create a simple pattern instead
            img = self.create_default_pattern(800, 800)
        
        img_array = np.array(img)
        
        # Flip vertically (image coordinates vs map coordinates)
        img_array = np.flipud(img_array)
        
        # Create occupancy grid
        self.grid = OccupancyGrid()
        self.grid.header.frame_id = 'world'
        self.grid.info.resolution = self.resolution
        self.grid.info.width = img_array.shape[1]
        self.grid.info.height = img_array.shape[0]
        self.grid.info.origin.position.x = self.origin_x
        self.grid.info.origin.position.y = self.origin_y
        self.grid.info.origin.position.z = 0.0
        self.grid.info.origin.orientation.w = 1.0
        
        # Convert to occupancy grid values (0-100)
        # Map full grayscale range to occupancy
        # 0 (black) -> 100 (occupied/dark)
        # 255 (white) -> 0 (free/light)
        occupancy = (100 - (img_array.astype(float) / 255.0 * 100.0)).astype(np.int8)
        
        # Flatten row-major (RViz expects this)
        self.grid.data = occupancy.flatten().tolist()
        
        # Check data range
        self.get_logger().info(f'Occupancy range: {min(self.grid.data)} to {max(self.grid.data)}')
        
        # Create QoS profile matching Map display expectations
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Publisher
        self.map_pub = self.create_publisher(
            OccupancyGrid, '/ground_map', map_qos
        )
        
        # Publish periodically
        self.timer = self.create_timer(1.0, self.publish_map)
        
        self.get_logger().info(f'Ground texture loaded: {img_array.shape[1]}x{img_array.shape[0]}')
        self.get_logger().info(f'Resolution: {self.resolution}m/pixel')
    
    def publish_map(self):
        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.grid)

    def create_default_pattern(self, width, height):
        """Create a simple checkerboard pattern if no image provided"""
        pattern = np.zeros((height, width), dtype=np.uint8)
        
        # Checkerboard
        square_size = 50
        for i in range(0, height, square_size):
            for j in range(0, width, square_size):
                if ((i // square_size) + (j // square_size)) % 2 == 0:
                    pattern[i:i+square_size, j:j+square_size] = 200  # Light
                else:
                    pattern[i:i+square_size, j:j+square_size] = 100  # Dark
        
        return Image.fromarray(pattern)

def main():
    rclpy.init()
    node = GroundTexturePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()