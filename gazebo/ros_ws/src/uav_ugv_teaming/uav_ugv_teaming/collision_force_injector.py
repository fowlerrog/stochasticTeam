#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from roscopter_msgs.msg import State
from nav_msgs.msg import Odometry
import math
from std_msgs.msg import Bool
import traceback

class CollisionForceInjector(Node):
    def __init__(self):
        super().__init__('collision_force_injector')
        
        # Declare parameters
        self.declare_parameter('ugv_landing_height', 0.3) # cylindrical size of UGV
        self.declare_parameter('ugv_landing_radius', 0.3)

        # Get parameters
        self.ugv_landing_height = self.get_parameter('ugv_landing_height').value
        self.ugv_landing_radius = self.get_parameter('ugv_landing_radius').value

        # Robot states
        self.uav_state = None
        self.ugv_odom = None
        
        # Collision parameters - vertical
        self.collision_stiffness = 2000.0  # N/m (spring constant for vertical)
        self.collision_damping = 1000.0  # N·s/m (damper for vertical)
        self.min_force = -10.0 # maximum downward force we can apply
        
        # Friction parameters - horizontal
        self.friction_coef_static = 0.8  # Static friction
        self.friction_coef_kinetic = 0.6  # Kinetic friction
        self.friction_transition_speed = 0.5  # m/s
        
        # Attitude stabilization parameters (torques to level out)
        self.roll_stiffness = 100.0  # N·m/rad (torque per radian of roll)
        self.roll_damping = 30.0  # N·m·s/rad (torque damping)
        self.pitch_stiffness = 100.0  # N·m/rad
        self.pitch_damping = 30.0  # N·m·s/rad
        self.yaw_damping = 15.0  # N·m·s/rad (only damping for yaw, no restoring)
        
        # Subscribers
        self.create_subscription(State, '/sim/roscopter/state',#'/estimated_state', 
                                lambda msg: setattr(self, 'uav_state', msg), 10)
        self.create_subscription(Odometry, '/ugv/odom',
                                lambda msg: setattr(self, 'ugv_odom', msg), 10)
        
        # Publishers for external forces and contact status
        self.force_pub = self.create_publisher(
            WrenchStamped,
            '/sim/forces_and_moments_collector', # '/sim/forces_and_moments',
            10
        )
        self.contact_pub = self.create_publisher(
            Bool,
            '/collision_force_injector/in_contact',
            10
        )

        # High rate for physics
        self.create_timer(0.01, self.compute_collision_forces)  # 100 Hz
        
        self.in_contact = False
        
        self.get_logger().info('Collision force injector started')
        self.get_logger().info(f'  Vertical: k={self.collision_stiffness}, c={self.collision_damping}')
        self.get_logger().info(f'  Friction: static={self.friction_coef_static}, kinetic={self.friction_coef_kinetic}')
        self.get_logger().info(f'  Attitude: roll/pitch k={self.roll_stiffness}, c={self.roll_damping}')
        self.get_logger().info(f'  Yaw damping: c={self.yaw_damping}')
    
    def compute_collision_forces(self):
        """Compute and publish collision forces and stabilizing torques"""
        if self.uav_state is None or self.ugv_odom is None:
            return
        
        # Convert positions to same frame (world/NED)
        ugv_x = self.ugv_odom.pose.pose.position.x
        ugv_y = self.ugv_odom.pose.pose.position.y
        ugv_z = self.ugv_odom.pose.pose.position.z

        uav_x_world = self.uav_state.p_n
        uav_y_world = -self.uav_state.p_e
        uav_z_world = -self.uav_state.p_d  # NED down to world up
        
        # Check if UAV is below UGV surface
        ugv_top = ugv_z + self.ugv_landing_height
        penetration = ugv_top - uav_z_world
        
        xy_dist = math.sqrt( (uav_x_world - ugv_x)**2 + (uav_y_world - ugv_y)**2 )

        if penetration > 0 and xy_dist < self.ugv_landing_radius:
            # ============ VERTICAL FORCES ============
            # Spring force (pushes UAV up)
            force_z = self.collision_stiffness * penetration
            
            # Damping force (opposes vertical motion)
            uav_vz_world = -self.uav_state.v_z  # NED down velocity to world up velocity
            force_z += self.collision_damping * (-uav_vz_world)
            
            # Ensure we're pushing up
            force_z = max(self.min_force, force_z)
            
            # ============ HORIZONTAL FRICTION FORCES ============
            # Compute relative velocity between UAV and UGV (in body frame)
            # Need to convert UGV velocity to UAV body frame
            ugv_vx_body = self.ugv_odom.twist.twist.linear.x  # forward
            ugv_vy_body = self.ugv_odom.twist.twist.linear.y  # left
            
            # Get UGV orientation (yaw)
            ugv_qz = self.ugv_odom.pose.pose.orientation.z
            ugv_qw = self.ugv_odom.pose.pose.orientation.w
            ugv_yaw = math.atan2(2.0 * ugv_qw * ugv_qz, 1.0 - 2.0 * ugv_qz * ugv_qz)

            cos_ugv_yaw = math.cos(-ugv_yaw)
            sin_ugv_yaw = math.sin(-ugv_yaw)

            # UGV velocity in inertial NED frame
            ugv_vn_inertial = cos_ugv_yaw * ugv_vx_body - sin_ugv_yaw * ugv_vy_body
            ugv_ve_inertial = sin_ugv_yaw * ugv_vx_body + cos_ugv_yaw * ugv_vy_body

            # Add in yaw rate cross lever arm
            ugv_yaw_rate = -self.ugv_odom.twist.twist.angular.z
            lever_arm_n = self.uav_state.p_n - self.ugv_odom.pose.pose.position.x
            lever_arm_e = -self.uav_state.p_e - self.ugv_odom.pose.pose.position.y
            ugv_vn_inertial += ugv_yaw_rate * lever_arm_e
            ugv_ve_inertial += -ugv_yaw_rate * lever_arm_n

            # Now convert inertial NED to UAV body frame
            uav_vx_body = self.uav_state.v_x
            uav_vy_body = self.uav_state.v_y

            # Convert inertial velocities to UAV body frame
            cos_uav_yaw = math.cos(self.uav_state.psi)
            sin_uav_yaw = math.sin(self.uav_state.psi)

            ugv_vx_uav_body = cos_uav_yaw * ugv_vn_inertial + sin_uav_yaw * ugv_ve_inertial
            ugv_vy_uav_body = -sin_uav_yaw * ugv_vn_inertial + cos_uav_yaw * ugv_ve_inertial

            # Relative velocity in UAV body frame
            rel_vx = uav_vx_body - ugv_vx_uav_body
            rel_vy = uav_vy_body - ugv_vy_uav_body
            rel_v_lateral = math.sqrt(rel_vx**2 + rel_vy**2)
            
            # Friction force magnitude
            normal_force = abs(force_z)
            
            if rel_v_lateral < self.friction_transition_speed:
                # Static friction (smooth transition)
                blend = rel_v_lateral / self.friction_transition_speed
                friction_coef = self.friction_coef_static * (1 - blend) + self.friction_coef_kinetic * blend
            else:
                # Kinetic friction
                friction_coef = self.friction_coef_kinetic
            
            friction_force_mag = friction_coef * normal_force
            
            if rel_v_lateral > 0.001:
                # Apply friction opposing relative motion
                force_x = -friction_force_mag * (rel_vx / rel_v_lateral)
                force_y = -friction_force_mag * (rel_vy / rel_v_lateral)
            else:
                # No relative motion - no friction needed
                force_x = 0.0
                force_y = 0.0
            
            # ============ STABILIZING TORQUES ============
            # Torque to level out roll (phi -> 0)
            torque_roll = -self.roll_stiffness * self.uav_state.phi - \
                         self.roll_damping * self.uav_state.p
            
            # Torque to level out pitch (theta -> 0)
            torque_pitch = -self.pitch_stiffness * self.uav_state.theta - \
                          self.pitch_damping * self.uav_state.q
            
            # Torque to damp yaw rotation (no restoring force)
            # This prevents sliding/spinning but doesn't force any particular yaw
            torque_yaw = -self.yaw_damping * (self.uav_state.r + self.ugv_odom.twist.twist.angular.z)
            
            # ============ SAFETY CHECKS ============
            # Check for NaN or Inf
            def safe_float(value, default=0.0):
                """Ensure value is a valid float"""
                if math.isnan(value) or math.isinf(value):
                    self.get_logger().warn(f'Invalid value detected: {value}, using {default}')
                    return default
                return float(value)
            
            force_x = safe_float(force_x)
            force_y = safe_float(force_y)
            force_z = safe_float(force_z)
            torque_roll = safe_float(torque_roll)
            torque_pitch = safe_float(torque_pitch)
            torque_yaw = safe_float(torque_yaw)
            
            # Clamp to reasonable ranges
            max_force = 1000.0  # N
            max_torque = 100.0  # Nm
            
            force_x = max(-max_force, min(max_force, force_x))
            force_y = max(-max_force, min(max_force, force_y))
            force_z = max(self.min_force, min(max_force, force_z))
            torque_roll = max(-max_torque, min(max_torque, torque_roll))
            torque_pitch = max(-max_torque, min(max_torque, torque_pitch))
            torque_yaw = max(-max_torque, min(max_torque, torque_yaw))

            # ============ PUBLISH WRENCH ============
            wrench = WrenchStamped()
            wrench.header.stamp = self.get_clock().now().to_msg()
            wrench.header.frame_id = 'body'  # Forces and torques in body frame
            
            # Forces (body frame: x=forward, y=right, z=down)
            wrench.wrench.force.x = force_x
            wrench.wrench.force.y = force_y
            wrench.wrench.force.z = -force_z  # World up to body down (NED)
            
            # Torques (body frame: roll, pitch, yaw)
            wrench.wrench.torque.x = torque_roll
            wrench.wrench.torque.y = torque_pitch
            wrench.wrench.torque.z = torque_yaw

            self.force_pub.publish(wrench)
            
            if not self.in_contact:
                self.get_logger().info('UAV in contact with UGV - applying forces/torques')
                self.in_contact = True

        else:
            # No contact
            if self.in_contact:
                self.get_logger().info('UAV lost contact with UGV')
                self.in_contact = False
            
            # # Publish zero wrench (or just stop publishing)
            # wrench = WrenchStamped()
            # wrench.header.stamp = self.get_clock().now().to_msg()
            # wrench.header.frame_id = 'body'
            # # All zeros
            # self.force_pub.publish(wrench)

        # Publish in contact
        in_contact_msg = Bool()
        in_contact_msg.data = self.in_contact
        self.contact_pub.publish(in_contact_msg)

def main():
    rclpy.init()
    node = CollisionForceInjector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()