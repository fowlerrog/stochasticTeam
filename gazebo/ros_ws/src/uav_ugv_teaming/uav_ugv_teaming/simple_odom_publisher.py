#!/usr/bin/env python3
"""
simple_odom_publisher.py - Lightweight odometry simulation for differential drive robot
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math

class SimpleOdomPublisher(Node):
    def __init__(self):
        super().__init__('simple_odom_publisher')
        
        # Get namespace
        self.namespace = self.get_namespace()
        if self.namespace == '/':
            self.namespace = ''

        # Robot parameters (TurtleBot4-like)
        self.wheel_base = 0.16  # meters between wheels
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vth = 0.0

        # TF frame names (with namespace)
        self.odom_frame = f'{self.namespace}/odom' if self.namespace else 'odom'
        self.base_link_frame = f'{self.namespace}/base_link' if self.namespace else 'base_link'
        
        # Remove leading slash if present
        self.odom_frame = self.odom_frame.lstrip('/')
        self.base_link_frame = self.base_link_frame.lstrip('/')

        # Subscriber to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for publishing odometry
        self.timer = self.create_timer(0.02, self.publish_odom)  # 50 Hz
        self.last_time = self.get_clock().now()
        
        self.get_logger().info(f'Simple Odometry Publisher started with namespace: {self.namespace}')
        self.get_logger().info(f'TF frames: {self.odom_frame} -> {self.base_link_frame}')
    
    def cmd_vel_callback(self, msg):
        """Update velocity commands"""
        self.vx = msg.linear.x
        self.vth = msg.angular.z
    
    def publish_odom(self):
        """Compute and publish odometry"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # Skip if dt is too small or too large
        if dt < 0.001 or dt > 1.0:
            self.last_time = current_time
            return
            
        self.last_time = current_time
        
        # Update pose based on differential drive kinematics
        delta_x = self.vx * math.cos(self.theta) * dt
        delta_y = self.vx * math.sin(self.theta) * dt
        delta_theta = self.vth * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Publish TF transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_link_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Convert theta to quaternion
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        
        # Debug: Log occasionally
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 0
            
        if self._debug_counter % 250 == 0:  # Every 5 seconds at 50Hz
            self.get_logger().info(f'Publishing TF: {self.odom_frame} -> {self.base_link_frame} at ({self.x:.2f}, {self.y:.2f})')
        
        self.tf_broadcaster.sendTransform(t)
        
        # Publish Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_link_frame
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = t.transform.rotation
        
        # Velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth
        
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
