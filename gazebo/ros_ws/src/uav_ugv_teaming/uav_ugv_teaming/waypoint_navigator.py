#!/usr/bin/env python3
"""
waypoint_navigator.py - Simple waypoint navigation without obstacle avoidance
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_srvs.srv import Trigger
import math
import ast

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Declare and evaluate parameters
        paramList = [
            ('waypoints', '[]'),
            ('linear_speed', 0.2),
            ('angular_speed', 0.5),
            ('distance_tolerance', 0.1),
            ('linear_accel', 0.3),
        ]
        for name, defaultValue in paramList:
            self.declare_parameter(name, defaultValue)
            setattr(self, name, self.get_parameter(name).value)
        
        # Parse waypoints string into list
        try:
            waypoints_list = ast.literal_eval(self.waypoints)
        except (ValueError, SyntaxError):
            self.get_logger().error(f'Failed to parse waypoints: {self.waypoints}')
            waypoints_list = []

        # Convert to list of tuples (x, y)
        self.waypoints = []
        if waypoints_list:
            for i in range(0, len(waypoints_list), 2):
                if i + 1 < len(waypoints_list):
                    try:
                        self.waypoints.append((float(waypoints_list[i]), float(waypoints_list[i+1])))
                    except:
                        self.get_logger().error(f'Invalid waypoint: {(waypoints_list[i], waypoints_list[i+1])}')
        
        self.current_waypoint_idx = 0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.last_speed = 0.0
        self.last_time = self.get_clock().now()
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # Subscribe to new waypoint topic (single waypoint)
        self.waypoint_sub = self.create_subscription(
            PoseStamped, 
            'waypoint', 
            self.waypoint_callback, 
            10
        )
        
        # Subscribe to path topic (multiple waypoints)
        self.path_sub = self.create_subscription(
            Path,
            'waypoint_path',
            self.path_callback,
            10
        )

        # Subscribe to clear path service
        self.clear_service = self.create_service(
            Trigger,
            'waypoint_clear',
            self.clear_waypoints_callback
        )
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f'Waypoint Navigator started with {len(self.waypoints)} waypoints')
        if self.waypoints:
            self.get_logger().info(f'First waypoint: ({self.waypoints[0][0]}, {self.waypoints[0][1]})')
        self.get_logger().info('Listening for waypoints on topics: /waypoint and /waypoint_path')

    def waypoint_callback(self, msg):
        """Add a single waypoint from PoseStamped message"""
        new_waypoint = (msg.pose.position.x, msg.pose.position.y)
        self.waypoints.append(new_waypoint)
        self.get_logger().info(f'Added waypoint: ({new_waypoint[0]:.2f}, {new_waypoint[1]:.2f}). Total: {len(self.waypoints)}')
    
    def path_callback(self, msg):
        """Replace all waypoints from Path message"""
        self.waypoints.clear()
        for pose in msg.poses:
            waypoint = (pose.pose.position.x, pose.pose.position.y)
            self.waypoints.append(waypoint)
        
        self.current_waypoint_idx = 0
        self.get_logger().info(f'Received new path with {len(self.waypoints)} waypoints')
        if self.waypoints:
            self.get_logger().info(f'First waypoint: ({self.waypoints[0][0]:.2f}, {self.waypoints[0][1]:.2f})')
    
    def clear_waypoints_callback(self, request, response):
        self.waypoints.clear()
        self.current_waypoint_idx = 0
        self.stop_robot()
        self.get_logger().info('Cleared all waypoints')
        response.success = True
        response.message = 'Cleared all waypoints'
        return response

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
        now = self.get_clock().now()
        if not self.waypoints or self.current_waypoint_idx >= len(self.waypoints):
            # No waypoints or finished all waypoints
            self.stop_robot()
            self.last_time = now
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
                self.last_time = now
            return
        
        # Simple proportional controller
        # First, orient towards the target
        if abs(angle_diff) > 0.1:  # 0.1 radians ~ 5.7 degrees
            # Turn in place
            cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            cmd.linear.x = 0.0
            self.last_speed = 0.0
        else:
            # Move forward
            dt = (now - self.last_time).to_msg().nanosec * 1e-9
            # Check for braking distance
            if distance + self.distance_tolerance <= 0.5 * self.last_speed**2 / self.linear_accel:
                self.last_speed = max([
                    0, # stop
                    self.last_speed - self.linear_accel * dt, # max brake
                    math.sqrt(2 * distance * self.linear_accel) # vel to constantly max brake
                ])
            else: # move as fast as possible
                self.last_speed = min(
                    self.linear_speed, # max speed
                    self.last_speed + self.linear_accel * dt # max accel
                )
            cmd.linear.x = min(self.last_speed, distance) # graceful final braking
            cmd.angular.z = 2.0 * angle_diff  # Proportional control for fine adjustment
        
        self.cmd_vel_pub.publish(cmd)
        self.last_time = now
    
    def stop_robot(self):
        """Stop the robot"""
        cmd = Twist()
        self.last_speed = 0.0
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
