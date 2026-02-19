#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class MeshScaler(Node):
    def __init__(self):
        super().__init__('mesh_scaler')
        
        # Declare parameters
        self.declare_parameter('input_topic', '/rviz/mesh')
        self.declare_parameter('output_topic', '/rviz/mesh_scaled')
        self.declare_parameter('scale', 1.0)

        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.scale = self.get_parameter('scale').value
                
        # Subscriber
        self.sub = self.create_subscription(
            Marker, self.input_topic,
            self.sub_callback, 10
        )
        
        # Publisher
        self.pub = self.create_publisher(Marker, self.output_topic, 10)

        self.get_logger().info('Mesh Scaler started')

    def sub_callback(self, msg):
        # Change scaling and echo topic
        msg.scale.x = self.scale
        msg.scale.y = self.scale
        msg.scale.z = self.scale
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = MeshScaler()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
