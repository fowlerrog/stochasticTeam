#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped

class WrenchCollector(Node):
    """Sums all forces and torques published to one topic, and publishes to another topic"""
    def __init__(self):
        super().__init__('wrench_collector')
        
        # Declare parameters
        self.declare_parameter('input_topic', '/sim/forces_and_moments_collector')
        self.declare_parameter('output_topic', '/sim/forces_and_moments')

        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        self.total_wrench = None

        # Subscriber
        self.sub = self.create_subscription(
            WrenchStamped, self.input_topic,
            self.input_callback, 10
        )
        
        # Publisher
        self.pub = self.create_publisher(WrenchStamped, self.output_topic, 10)

        # Timer
        self.timer = self.create_timer(0.0025, self.update)  # 400 Hz

        self.get_logger().info('Wrench Collector started')

    def input_callback(self, msg):
        if self.total_wrench is None:
            self.total_wrench = msg
        else:
            self.total_wrench.wrench.force.x += msg.wrench.force.x
            self.total_wrench.wrench.force.y += msg.wrench.force.y
            self.total_wrench.wrench.force.z += msg.wrench.force.z
            self.total_wrench.wrench.torque.x += msg.wrench.torque.x
            self.total_wrench.wrench.torque.y += msg.wrench.torque.y
            self.total_wrench.wrench.torque.z += msg.wrench.torque.z

    def update(self):
        if self.total_wrench is not None:
            self.pub.publish(self.total_wrench)
            self.total_wrench = None

def main():
    rclpy.init()
    node = WrenchCollector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
