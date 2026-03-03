#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from roscopter_msgs.msg import State
from rosflight_msgs.msg import Command, Status
from std_srvs.srv import SetBool, Trigger

class TakeoffCommander(Node):
    def __init__(self):
        super().__init__('takeoff_commander')
        
        # Declare and evaluate parameters
        paramList = [
            ('raw_command_topic', '/raw_command'),
            ('uav_odom_topic', '/estimated_state'),
            ('command_topic', '/command'),
            ('activate_takeoff_service', '/takeoff_commader/toggle'),
            ('uav_override_service', '/toggle_override'),
            ('takeoff_throttle_coefficient', 0.2),
            ('ugv_landing_height', 0.3),
            ('coarse_distance_tolerance', 0.7)
        ]
        for name, defaultValue in paramList:
            self.declare_parameter(name, defaultValue)
            setattr(self, name, self.get_parameter(name).value)

        # State
        self.takeoff_active = False
        self.takeoff_start_position = None
        self.takeoff_start_yaw = None
        self.current_uav_state = None
        self.last_command = None

        # Subscribers
        self.cmd_sub = self.create_subscription(
            Command,
            self.raw_command_topic,
            self.cmd_callback,
            10
        )        
        self.state_sub = self.create_subscription(
            State,
            self.uav_odom_topic,
            self.state_callback,
            10
        )
        
        # Publisher
        self.cmd_pub = self.create_publisher(
            Command,
            self.command_topic,
            10
        )
        
        # Service to toggle takeoff mode
        self.takeoff_srv = self.create_service(
            SetBool,
            self.activate_takeoff_service,
            self.takeoff_service_callback
        )

        # Client to override UAV control
        self.uav_override_cli = self.create_client(
            Trigger, self.uav_override_service
        )

        # Timer to publish at consistent rate
        self.timer = self.create_timer(0.0025, self.publish_command)  # 400 Hz

        self.get_logger().info('Takeoff Commander started')
        self.get_logger().info(f'  Takeoff Throttle coeff: {self.takeoff_throttle_coefficient}')
        self.get_logger().info( '  Service: /activate_takeoff (std_srvs/SetBool)')
        self.get_logger().info(f'  Input: {self.raw_command_topic}')
        self.get_logger().info(f'  Output: {self.command_topic}')
    
    def cmd_callback(self, msg):
        """Store the latest command"""
        self.last_command = msg
    
    def state_callback(self, msg):
        """Store current UAV state"""
        self.current_uav_state = msg

    def takeoff_service_callback(self, request, response):
        """Toggle takeoff mode via service"""
        if request.data:
            # Activate takeoff
            if self.current_uav_state is None:
                response.success = False
                response.message = 'Cannot activate takeoff - no UAV state available'
                self.get_logger().error(response.message)
                return response
            
            self.takeoff_active = True
            
            # Capture current position and yaw
            self.takeoff_start_position = {
                'n': float(self.current_uav_state.p_n),
                'e': float(self.current_uav_state.p_e),
                'd': float(self.current_uav_state.p_d)
            }
            self.takeoff_start_yaw = float(self.current_uav_state.psi)
            
            response.success = True
            response.message = (
                f'takeoff activated at N={self.takeoff_start_position["n"]:.2f}, '
                f'E={self.takeoff_start_position["e"]:.2f}, '
                f'D={self.takeoff_start_position["d"]:.2f}, '
                f'Yaw={self.takeoff_start_yaw:.2f}'
            )
            self.get_logger().info(response.message)

            # Enable UAV control
            while not self.uav_override_cli.wait_for_service(timeout_sec=1.0):
                pass
            self.uav_override_cli.call_async(Trigger.Request())

        else:
            # Deactivate takeoff
            self.takeoff_active = False
            self.takeoff_start_position = None
            self.takeoff_start_yaw = None
            self.last_command = None
            
            response.success = True
            response.message = 'takeoff deactivated - passing through commands'
            self.get_logger().info(response.message)
        
        return response
    
    def publish_command(self):
        """Publish command - either passthrough or takeoff override"""

        # Check for takeoff first
        if self.takeoff_active and \
        self.current_uav_state is not None and \
        -self.current_uav_state.p_d > self.ugv_landing_height + self.coarse_distance_tolerance:
            self.get_logger().info('Detected successful takeoff; returning to normal control')
            self.takeoff_active = False
            return

        if not self.takeoff_active:
            # Passthrough mode - echo input to output
            if self.last_command is not None:
                self.cmd_pub.publish(self.last_command)
        else:
            # takeoff mode - generate descent command
            if self.takeoff_start_position is None:
                self.get_logger().warn('takeoff active but no start position - skipping')
                return
            
            # Create takeoff command (Mode 4: north pos, east pos, down vel, yaw)
            cmd = Command()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'NED'
            
            cmd.mode = Command.MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE
            cmd.u = [0.0] * 10
            # cmd.u[2] = self.takeoff_throttle # set only throttle command [0-1]
            # zero thrust at >= 3 m + ugv height, with linear ramp up to K at ugv height
            cmd.u[2] = min(self.takeoff_throttle_coefficient, max(0,
                self.takeoff_throttle_coefficient * (3 + self.current_uav_state.p_d - self.ugv_landing_height)
            ))
            
            self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = TakeoffCommander()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()