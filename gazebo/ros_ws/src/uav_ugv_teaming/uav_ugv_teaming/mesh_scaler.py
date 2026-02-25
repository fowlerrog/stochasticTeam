#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class MeshScaler(Node):
    def __init__(self):
        super().__init__('mesh_scaler')

        # Declare and evaluate parameters
        paramList = [
            ('input_topic', '/rviz/mesh'),
            ('output_topic', '/rviz/mesh_scaled'),
            ('scale', 1.0),
            ('color', [1.0, 0.25, 0.7, 1.0]),
        ]
        for name, defaultValue in paramList:
            self.declare_parameter(name, defaultValue)
            setattr(self, name, self.get_parameter(name).value)
                
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
        msg.color.r = self.color[0]
        msg.color.g = self.color[1]
        msg.color.b = self.color[2]
        msg.color.a = self.color[3]
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = MeshScaler()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
