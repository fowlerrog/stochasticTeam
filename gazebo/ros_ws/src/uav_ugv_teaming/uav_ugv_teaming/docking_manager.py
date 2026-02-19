#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from roscopter_msgs.msg import State

class DockingManager(Node):
    def __init__(self):
        super().__init__('docking_manager')
        
        # Declare parameters
        self.declare_parameter('uav_odom_topic', '/estimated_state')
        self.declare_parameter('uav_truth_odom_topic', '/sim/roscopter/state')
        self.declare_parameter('uav_odom_output_topic', '/estimated_state_with_docking')
        self.declare_parameter('ugv_odom_topic', '/ugv/odom')
        self.declare_parameter('fine_distance_tolerance', 0.1)
        self.declare_parameter('ugv_landing_height', 0.5) # UGV landing offset
        self.declare_parameter('uav_base_link', '/stl_frame')
        self.declare_parameter('ugv_base_link', '/ugv/base_link')

        # Get parameters
        self.uav_odom_topic = self.get_parameter('uav_odom_topic').value
        self.uav_truth_odom_topic = self.get_parameter('uav_truth_odom_topic').value
        self.uav_odom_output_topic = self.get_parameter('uav_odom_output_topic').value
        self.ugv_odom_topic = self.get_parameter('ugv_odom_topic').value
        self.fine_distance_tolerance = self.get_parameter('fine_distance_tolerance').value
        self.ugv_landing_height = self.get_parameter('ugv_landing_height').value
        self.uav_base_link = self.get_parameter('uav_base_link').value
        self.ugv_base_link = self.get_parameter('ugv_base_link').value

        # Docking state
        self.is_docked = False
        self.allow_docking = False
        
        # Robot positions
        self.uav_odom_raw = None  # Raw UAV position from its own dynamics
        self.uav_truth_odom = None
        self.ugv_odom = None
                
        # Subscribers
        self.uav_odom_sub = self.create_subscription(
            State, self.uav_odom_topic,  # Subscribe to raw UAV odom
            lambda msg: setattr(self, 'uav_odom_raw', msg), 10
        )
        self.uav_truth_odom_sub = self.create_subscription(
            State, self.uav_truth_odom_topic,
            lambda msg: setattr(self, 'uav_truth_odom', msg), 10
        )
        self.ugv_odom_sub = self.create_subscription(
            Odometry, self.ugv_odom_topic,
            lambda msg: setattr(self, 'ugv_odom', msg), 10
        )
        
        # Publisher for corrected UAV odometry
        self.uav_odom_pub = self.create_publisher(State, self.uav_odom_output_topic, 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publish docking state
        self.is_docked_pub = self.create_publisher(Bool, '/docking_manager/is_docked', 10)

        # Services to allow and disallow dockings
        self.allow_docking_service = self.create_service(
            Trigger,
            '/docking_manager/allow_docking',
            self.allow_docking_callback
        )
        self.disallow_docking_service = self.create_service(
            Trigger,
            '/docking_manager/disallow_docking',
            self.disallow_docking_callback
        )

        # Timer
        self.timer = self.create_timer(0.02, self.update)  # 50 Hz
        
        self.get_logger().info('UAV Docking Manager started')

    def allow_docking_callback(self, request, response):
        self.allow_docking = True
        msgString = 'Set allow_docking to True'
        self.get_logger().info(msgString)
        response.success = True
        response.message = msgString
        return response

    def disallow_docking_callback(self, request, response):
        self.allow_docking = False
        msgString = 'Set allow_docking to True'
        if self.is_docked:
            self.is_docked = False
            msgString += ' and undocked'
        self.get_logger().info(msgString)
        response.success = True
        response.message = msgString
        return response

    def check_docking_conditions(self):
        """Check if UAV should transition to/from docked state"""
        if self.ugv_odom is None or self.uav_odom_raw is None or \
            not self.allow_docking or self.uav_truth_odom is None:
            return
        
        # Calculate relative position
        # dx = self.uav_truth_odom.p_n - self.ugv_odom.pose.pose.position.x
        # dy = -self.uav_truth_odom.p_e - self.ugv_odom.pose.pose.position.y
        # dz = -self.uav_truth_odom.p_d - self.ugv_odom.pose.pose.position.z - self.ugv_landing_height
        dx = self.uav_odom_raw.p_n - self.ugv_odom.pose.pose.position.x
        dy = -self.uav_odom_raw.p_e - self.ugv_odom.pose.pose.position.y
        dz = -self.uav_odom_raw.p_d - self.ugv_odom.pose.pose.position.z - self.ugv_landing_height

        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # State transitions (only allow docking, because undocking occurs by setting allow_docking to False)
        if not self.is_docked:
            # Check if UAV is landing
            if dist < self.fine_distance_tolerance:
                self.is_docked = True
                self.get_logger().info('UAV DOCKED to UGV')
    
    def update(self):
        """Main update loop"""
        if self.ugv_odom is None or self.uav_odom_raw is None or self.uav_truth_odom is None:
            return
        
        # Update docking state
        self.check_docking_conditions()
        is_docked_msg = Bool()
        is_docked_msg.data = self.is_docked
        self.is_docked_pub.publish(is_docked_msg)
        
        if self.is_docked:
            # Create modified odometry with UGV position + offset
            modified_odom = Odometry()
            modified_odom.header.stamp = self.get_clock().now().to_msg()
            modified_odom.header.frame_id = 'world'
            
            # Position: UGV + offset
            modified_odom.pose.pose.position.x = self.ugv_odom.pose.pose.position.x
            modified_odom.pose.pose.position.y = self.ugv_odom.pose.pose.position.y
            modified_odom.pose.pose.position.z = self.ugv_odom.pose.pose.position.z + self.ugv_landing_height
            
            # Orientation: match UGV (keep level)
            modified_odom.pose.pose.orientation = self.ugv_odom.pose.pose.orientation
            
            # Velocity: match UGV
            modified_odom.twist.twist = self.ugv_odom.twist.twist
            
            # Convert to State message
            corrected_state = self.odom_to_state(modified_odom, self.uav_odom_raw)

            # Publish TF showing docking
            # self.publish_docking_tf()
        else:
            # UAV is flying - use its own odometry
            # corrected_state = self.uav_truth_odom
            # corrected_state.initial_lat = self.uav_odom_raw.initial_lat
            # corrected_state.initial_lon = self.uav_odom_raw.initial_lon
            # corrected_state.initial_alt = self.uav_odom_raw.initial_alt
            corrected_state = self.uav_odom_raw

        # Publish corrected odometry
        self.uav_odom_pub.publish(corrected_state)

    def publish_docking_tf(self):
        """Publish TF showing UAV docked on UGV"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.ugv_base_link
        t.child_frame_id = self.uav_base_link
        
        # UAV is directly above UGV
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = self.ugv_landing_height
        
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)
    
    def get_docking_state(self):
        """Public method to check docking state"""
        return self.is_docked

    def odom_to_state(self, odom_msg, base_state):
        """
        Convert nav_msgs/Odometry (ENU) to roscopter_msgs/State (NED)
        Copies non-kinematic fields from base_state
        """
        state = State()
        
        # Header
        state.header = odom_msg.header
        # state.header.frame_id = ''
        
        # Position: Convert NWU to NED
        # NWU: x=North, y=West, z=Up
        # NED: p_n=North, p_e=East, p_d=Down
        state.p_n = odom_msg.pose.pose.position.x       # North = NWU x
        state.p_e = -odom_msg.pose.pose.position.y      # East = -NWU y (West→East)
        state.p_d = -odom_msg.pose.pose.position.z      # Down = -NWU z (Up→Down)
        
        # Velocity: Convert NWU to NED body frame
        # Assuming body frame velocities follow same convention
        state.v_x = odom_msg.twist.twist.linear.x       # Body x velocity (forward/north)
        state.v_y = -odom_msg.twist.twist.linear.y      # Body y velocity (right/east)
        state.v_z = -odom_msg.twist.twist.linear.z      # Body z velocity (down)
        
        # Angular rates (body frame)
        state.p = odom_msg.twist.twist.angular.x        # Roll rate
        state.q = -odom_msg.twist.twist.angular.y       # Pitch rate (flip)
        state.r = -odom_msg.twist.twist.angular.z       # Yaw rate (flip)
        
        # Orientation: Convert quaternion NWU to NED
        qx = odom_msg.pose.pose.orientation.x
        qy = odom_msg.pose.pose.orientation.y
        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w
        
        # NWU to NED: negate y and z components
        state.quat.w = qw
        state.quat.x = qx
        state.quat.y = -qy
        state.quat.z = -qz
        
        # Extract Euler angles from quaternion (NED frame)
        state.phi, state.theta, state.psi = self.quaternion_to_euler_ned(state.quat)
        
        # Copy fields from base_state that aren't in Odometry
        state.b_x = base_state.b_x
        state.b_y = base_state.b_y
        state.b_z = base_state.b_z
        state.initial_lat = base_state.initial_lat
        state.initial_lon = base_state.initial_lon
        state.initial_alt = base_state.initial_alt
        
        return state
    
    def quaternion_to_euler_ned(self, quat):
        """Convert quaternion to Euler angles in NED frame"""
        # Roll (phi)
        sinr_cosp = 2 * (quat.w * quat.x + quat.y * quat.z)
        cosr_cosp = 1 - 2 * (quat.x * quat.x + quat.y * quat.y)
        phi = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (theta)
        sinp = 2 * (quat.w * quat.y - quat.z * quat.x)
        if abs(sinp) >= 1:
            theta = math.copysign(math.pi / 2, sinp)
        else:
            theta = math.asin(sinp)
        
        # Yaw (psi)
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        psi = math.atan2(siny_cosp, cosy_cosp)
        
        return phi, theta, psi

def main():
    rclpy.init()
    node = DockingManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
