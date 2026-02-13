#!/usr/bin/env python3
"""
waypoint_navigator.py - Simple waypoint navigation without obstacle avoidance
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import ast

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Declare parameters
        self.declare_parameter('waypoints', '[]')
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('distance_tolerance', 0.1)
        
        # Get parameters
        waypoints_str = self.get_parameter('waypoints').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        
        # Parse waypoints string into list
        try:
            waypoints_list = ast.literal_eval(waypoints_str)
        except (ValueError, SyntaxError):
            self.get_logger().error(f'Failed to parse waypoints: {waypoints_str}')
            waypoints_list = []
        
        # Convert to list of tuples (x, y)
        self.waypoints = []
        if waypoints_list:
            for i in range(0, len(waypoints_list), 2):
                if i + 1 < len(waypoints_list):
                    self.waypoints.append((waypoints_list[i], waypoints_list[i+1]))
        
        self.current_waypoint_idx = 0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f'Waypoint Navigator started with {len(self.waypoints)} waypoints')
        if self.waypoints:
            self.get_logger().info(f'First waypoint: ({self.waypoints[0][0]}, {self.waypoints[0][1]})')
    
    def odom_callback(self, msg):
        """Update robot pose from odometry"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Convert quaternion to euler angle
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.robot_theta = math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))
    
    def control_loop(self):
        """Main control loop for waypoint navigation"""
        if not self.waypoints or self.current_waypoint_idx >= len(self.waypoints):
            # No waypoints or finished all waypoints
            self.stop_robot()
            return
        
        # Get current target waypoint
        target_x, target_y = self.waypoints[self.current_waypoint_idx]
        
        # Calculate distance and angle to target
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        # Calculate angle difference
        angle_diff = target_angle - self.robot_theta
        # Normalize to [-pi, pi]
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        cmd = Twist()
        
        # Check if we've reached the waypoint
        if distance < self.distance_tolerance:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)}')
            self.current_waypoint_idx += 1
            
            if self.current_waypoint_idx >= len(self.waypoints):
                self.get_logger().info('All waypoints reached!')
                self.stop_robot()
            return
        
        # Simple proportional controller
        # First, orient towards the target
        if abs(angle_diff) > 0.1:  # 0.1 radians ~ 5.7 degrees
            # Turn in place
            cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            cmd.linear.x = 0.0
        else:
            # Move forward
            cmd.linear.x = min(self.linear_speed, distance)
            cmd.angular.z = 2.0 * angle_diff  # Proportional control for fine adjustment
        
        self.cmd_vel_pub.publish(cmd)
    
    def stop_robot(self):
        """Stop the robot"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
