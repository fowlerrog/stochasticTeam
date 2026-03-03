#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys

def main():
    if len(sys.argv) < 3:
        print("Usage: ros2 run uav_ugv_teaming send_waypoint <x> <y> [namespace]")
        print("Example: ros2 run uav_ugv_teaming send_waypoint 3.0 2.0 ugv")
        return
    
    rclpy.init()
    node = Node('waypoint_sender')
    
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    namespace = sys.argv[3] if len(sys.argv) > 3 else ''
    
    topic = f'/{namespace}/waypoint' if namespace else '/waypoint'
    pub = node.create_publisher(PoseStamped, topic, 10)
    
    msg = PoseStamped()
    msg.header.frame_id = 'world'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.w = 1.0
    
    # Wait for subscriber
    node.get_logger().info(f'Waiting for subscriber on {topic}...')
    while pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    pub.publish(msg)
    node.get_logger().info(f"Sent waypoint: ({x}, {y}) to {topic}")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
