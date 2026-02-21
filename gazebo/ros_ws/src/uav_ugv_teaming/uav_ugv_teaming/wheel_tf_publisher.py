#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class WheelTFPublisher(Node):
    def __init__(self):
        super().__init__('wheel_tf_publisher')

        # Declare and evaluate parameters
        paramList = [
            ('ugv_odom_topic', '/ugv/odom'),
            ('wheel_base', 0.4),  # Distance between left/right wheels [m]
            ('wheel_track', 0.5),  # Distance between front/rear wheels [m]
            ('wheel_radius', 0.1), # Wheel radius [m]
            ('wheel_vertical_offset', -0.1), # Wheel center above base_link [m]
            ('ugv_base_link', '/ugv/base_link') 
        ]
        for name, defaultValue in paramList:
            self.declare_parameter(name, defaultValue)
            setattr(self, name, self.get_parameter(name).value)

        # Wheel positions (x, y) relative to base_link
        # Front-left, Front-right, Rear-left, Rear-right
        self.wheel_positions = {
            'front_left': (self.wheel_track / 2, self.wheel_base / 2),
            'front_right': (self.wheel_track / 2, -self.wheel_base / 2),
            'rear_left': (-self.wheel_track / 2, self.wheel_base / 2),
            'rear_right': (-self.wheel_track / 2, -self.wheel_base / 2),
        }
        
        # Wheel rotation angles (radians)
        self.wheel_angles = {
            'front_left': 0.0,
            'front_right': 0.0,
            'rear_left': 0.0,
            'rear_right': 0.0,
        }
        
        # Last update time for integration
        self.last_time = None
        
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            self.ugv_odom_topic,
            self.odom_callback,
            10
        )
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('Wheel TF Publisher started')
        self.get_logger().info(f'  Wheel base: {self.wheel_base}m')
        self.get_logger().info(f'  Wheel track: {self.wheel_track}m')
        self.get_logger().info(f'  Wheel radius: {self.wheel_radius}m')
        self.get_logger().info(f'  Wheel vertical offset: {self.wheel_vertical_offset}m')
    
    def odom_callback(self, msg):
        """Process odometry and publish wheel transforms"""
        current_time = self.get_clock().now()
        
        # Get velocities from twist
        linear_vel = msg.twist.twist.linear.x  # Forward velocity (m/s)
        angular_vel = msg.twist.twist.angular.z  # Yaw rate (rad/s)
        
        # Calculate dt for wheel rotation integration
        if self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds / 1e9
            
            # Calculate wheel velocities for differential drive
            # v_left = v_linear - (angular_vel * wheel_base / 2)
            # v_right = v_linear + (angular_vel * wheel_base / 2)
            v_left = linear_vel - (angular_vel * self.wheel_base / 2)
            v_right = linear_vel + (angular_vel * self.wheel_base / 2)
            
            # Update wheel angles (rotation = distance / radius)
            # Left wheels
            delta_angle_left = (v_left * dt) / self.wheel_radius
            self.wheel_angles['front_left'] += delta_angle_left
            self.wheel_angles['rear_left'] += delta_angle_left
            
            # Right wheels
            delta_angle_right = (v_right * dt) / self.wheel_radius
            self.wheel_angles['front_right'] += delta_angle_right
            self.wheel_angles['rear_right'] += delta_angle_right
            
            # Keep angles in reasonable range
            for wheel in self.wheel_angles:
                self.wheel_angles[wheel] = self.wheel_angles[wheel] % (2 * math.pi)
        
        self.last_time = current_time
        
        # Publish transforms for each wheel
        for wheel_name, (x, y) in self.wheel_positions.items():
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.ugv_base_link
            t.child_frame_id = f'ugv/{wheel_name}_wheel_link'
            
            # Position relative to base_link
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = self.wheel_vertical_offset
            
            # Rotation: wheels rotate around Y axis (in robot frame)
            # Convert rotation angle to quaternion
            angle = self.wheel_angles[wheel_name]
            
            # Quaternion for rotation around Y axis
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = math.sin(angle / 2.0)
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = math.cos(angle / 2.0)
            
            self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = WheelTFPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()