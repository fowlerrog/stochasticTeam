#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from roscopter_msgs.msg import State

class StateOverride(Node):
    def __init__(self):
        super().__init__('state_override')
        
        # Declare parameters
        self.declare_parameter('uav_odom_topic', '/estimated_state')
        self.declare_parameter('uav_truth_odom_topic', '/sim/roscopter/state')
        self.declare_parameter('uav_odom_output_topic', '/estimated_state_override')

        # Get parameters
        self.uav_odom_topic = self.get_parameter('uav_odom_topic').value
        self.uav_truth_odom_topic = self.get_parameter('uav_truth_odom_topic').value
        self.uav_odom_output_topic = self.get_parameter('uav_odom_output_topic').value
        
        # Robot positions
        self.uav_odom_raw = None  # Raw UAV position from its own dynamics
        self.uav_truth_odom = None # Truth odom
        self.prev_velocity = None # prev. truth velocity
        self.prev_time = None # prev. truth time
                
        # Subscribers
        self.uav_odom_sub = self.create_subscription(
            State, self.uav_odom_topic,  # Subscribe to raw UAV odom
            lambda msg: setattr(self, 'uav_odom_raw', msg), 10
        )
        self.uav_truth_odom_sub = self.create_subscription(
            State, self.uav_truth_odom_topic,
            self.uav_truth_odom_callback, 10
        )
        
        # Publisher for corrected UAV odometry
        self.uav_odom_pub = self.create_publisher(State, self.uav_odom_output_topic, 10)
        
        self.get_logger().info('UAV Docking Manager started')

    def uav_truth_odom_callback(self, msg):
        # Repeat truth odom, with uav_odom initial conditions
        if self.uav_odom_raw is None:
            return
        
        corrected_odom = msg
        corrected_odom.initial_lat = self.uav_odom_raw.initial_lat
        corrected_odom.initial_lon = self.uav_odom_raw.initial_lon
        corrected_odom.initial_alt = self.uav_odom_raw.initial_alt

        # fill in body-frame forces
        corrected_odom.b_x = self.uav_odom_raw.b_x
        corrected_odom.b_y = self.uav_odom_raw.b_y
        corrected_odom.b_z = self.uav_odom_raw.b_z

        # if self.prev_velocity is not None and self.prev_time is not None:
        #     new_time = (msg.header.stamp.sec + msg.header.stamp.nanosec / 1e-9)
        #     dt = new_time - self.prev_time
        #     corrected_odom.b_x = (msg.v_x - self.prev_velocity[0]) / dt
        #     corrected_odom.b_y = (msg.v_y - self.prev_velocity[1]) / dt
        #     corrected_odom.b_z = (msg.v_z - self.prev_velocity[2]) / dt
        # self.prev_time = (msg.header.stamp.sec + msg.header.stamp.nanosec / 1e-9)
        # self.prev_velocity = [msg.v_x, msg.v_y, msg.v_z]

        # Publish corrected odometry
        self.uav_odom_pub.publish(corrected_odom)

def main():
    rclpy.init()
    node = StateOverride()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
