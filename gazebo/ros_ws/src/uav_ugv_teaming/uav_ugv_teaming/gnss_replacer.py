#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from roscopter_msgs.msg import State
from rosflight_msgs.msg import GNSS
import math

class GnssReplacer(Node):
    def __init__(self):
        super().__init__('gnss_replacer')
        
        # Declare parameters
        self.declare_parameter('uav_odom_topic', '/estimated_state')
        self.declare_parameter('uav_truth_odom_topic', '/sim/roscopter/state')
        self.declare_parameter('gnss_topic', '/gnss')

        # Get parameters
        self.uav_odom_topic = self.get_parameter('uav_odom_topic').value
        self.uav_truth_odom_topic = self.get_parameter('uav_truth_odom_topic').value
        self.gnss_topic = self.get_parameter('gnss_topic').value

        # Reference point (from /estimated_state)
        self.initial_lat = None
        self.initial_lon = None
        self.initial_alt = None
        
        self.gnss_truth = None
        self.uav_odom = None

        # Earth radius in meters (matching roscopter)
        self.EARTH_RADIUS = 6378145.0
        self.PI = 3.1415926535
        
        # Subscribe to truth
        self.truth_sub = self.create_subscription(
            State,
            self.uav_truth_odom_topic,
            self.truth_callback,
            10
        )
        
        # Subscribe to real UAV odom for initial conditions
        self.uav_sub = self.create_subscription(
            State,
            self.uav_odom_topic,
            lambda msg: setattr(self, 'uav_odom', msg),
            10
        )

        # Publish fake GPS at a higher rate to dominate EKF
        self.gnss_pub = self.create_publisher(
            GNSS,
            self.gnss_topic,
            100
        )
        
        self.timer = self.create_timer(0.01, self.publish_truth_gnss)
        self.get_logger().info('Converting truth to perfect GNSS')

    def truth_callback(self, msg):
        """Convert truth state to GNSS message"""
        # first check if we have a reference point from real gnss
        if self.uav_odom is None:
            return
        
        self.initial_lat = self.uav_odom.initial_lat
        self.initial_lon = self.uav_odom.initial_lon
        self.initial_alt = self.uav_odom.initial_alt

        gnss_msg = GNSS()
        gnss_msg.header.stamp = self.get_clock().now().to_msg()
        gnss_msg.header.frame_id = 'NED'
        
        # this model appears to match roscopter's

        # Convert NED position (meters) to lat/lon/alt
        # North-South: 1 degree latitude ≈ 111,111 meters
        # East-West: 1 degree longitude ≈ 111,111 * cos(latitude) meters
        
        lat_rad = math.radians(self.initial_lat)
        
        # # Calculate lat/lon from NED offset
        # dlat = msg.p_n / 111111.0  # meters north to degrees
        # dlon = msg.p_e / (111111.0 * math.cos(lat_rad))  # meters east to degrees
        
        # gnss_msg.lat = self.initial_lat + dlat
        # gnss_msg.lon = self.initial_lon + dlon
        # gnss_msg.alt = self.initial_alt - msg.p_d  # NED: down is positive, alt is up
        
        # conversion formula taken from gazebo/ros_ws/src/rosflight_ros_pkgs/rosflight_sim/simulators/standalone_sim/src/standalone_sensors.cpp
        gnss_msg.lat = 180.0 / (self.EARTH_RADIUS * self.PI) * msg.p_n + self.initial_lat
        gnss_msg.lon = 180.0 / (self.EARTH_RADIUS * self.PI) / math.cos(self.initial_lat * self.PI / 180.0) * msg.p_e + self.initial_lon
        gnss_msg.alt = -msg.p_d + self.initial_alt

        # Perfect GPS - very low noise
        gnss_msg.horizontal_accuracy = 0.01  # 1cm accuracy
        gnss_msg.vertical_accuracy = 0.01
        
        # Velocity in NED frame
        gnss_msg.vel_n = msg.v_x  # Assuming body frame ≈ NED when level
        gnss_msg.vel_e = msg.v_y
        gnss_msg.vel_d = msg.v_z
        gnss_msg.speed_accuracy = 0.01
        
        # GPS status
        gnss_msg.fix_type = 3  # 3D fix
        gnss_msg.num_sat = 15
        
        # Timestamp (Unix time)
        gnss_msg.gnss_unix_seconds = msg.header.stamp.sec
        gnss_msg.gnss_unix_nanos = msg.header.stamp.nanosec

        self.gnss_truth = gnss_msg

    def publish_truth_gnss(self):
        if self.gnss_truth is None:
            return

        self.gnss_pub.publish(self.gnss_truth)

def main():
    rclpy.init()
    node = GnssReplacer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
