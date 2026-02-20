#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from roscopter_msgs.msg import State
from rosflight_msgs.msg import Command
from std_srvs.srv import SetBool

class LandingCommander(Node):
    def __init__(self):
        super().__init__('landing_commander')
        
        # Declare and evaluate parameters
        paramList = [
            ('descent_rate', 0.1),  # m/s (positive = down)
            ('raw_command_topic', '/raw_command'),
            ('uav_odom_topic', '/estimated_state'),
            ('command_topic', '/command'),
            ('activate_landing_service', '/landing_commader/toggle'),
        ]
        for name, defaultValue in paramList:
            self.declare_parameter(name, defaultValue)
            setattr(self, name, self.get_parameter(name).value)

        # State
        self.landing_active = False
        self.landing_start_position = None
        self.landing_start_yaw = None
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
        
        # Service to toggle landing mode
        self.landing_srv = self.create_service(
            SetBool,
            self.activate_landing_service,
            self.landing_service_callback
        )
        
        # Timer to publish at consistent rate
        self.timer = self.create_timer(0.0025, self.publish_command)  # 400 Hz
        
        self.get_logger().info('Landing Commander started')
        self.get_logger().info(f'  Descent rate: {self.descent_rate} m/s')
        self.get_logger().info( '  Service: /activate_landing (std_srvs/SetBool)')
        self.get_logger().info(f'  Input: {self.raw_command_topic}')
        self.get_logger().info(f'  Output: {self.command_topic}')
    
    def cmd_callback(self, msg):
        """Store the latest command"""
        self.last_command = msg
    
    def state_callback(self, msg):
        """Store current UAV state"""
        self.current_uav_state = msg
    
    def landing_service_callback(self, request, response):
        """Toggle landing mode via service"""
        if request.data:
            # Activate landing
            if self.current_uav_state is None:
                response.success = False
                response.message = 'Cannot activate landing - no UAV state available'
                self.get_logger().error(response.message)
                return response
            
            self.landing_active = True
            
            # Capture current position and yaw
            self.landing_start_position = {
                'n': float(self.current_uav_state.p_n),
                'e': float(self.current_uav_state.p_e),
                'd': float(self.current_uav_state.p_d)
            }
            self.landing_start_yaw = float(self.current_uav_state.psi)
            
            response.success = True
            response.message = (
                f'Landing activated at N={self.landing_start_position["n"]:.2f}, '
                f'E={self.landing_start_position["e"]:.2f}, '
                f'D={self.landing_start_position["d"]:.2f}, '
                f'Yaw={self.landing_start_yaw:.2f}'
            )
            self.get_logger().info(response.message)
            
        else:
            # Deactivate landing
            self.landing_active = False
            self.landing_start_position = None
            self.landing_start_yaw = None
            
            response.success = True
            response.message = 'Landing deactivated - passing through commands'
            self.get_logger().info(response.message)
        
        return response
    
    def publish_command(self):
        """Publish command - either passthrough or landing override"""
        
        if not self.landing_active:
            # Passthrough mode - echo input to output
            if self.last_command is not None:
                self.cmd_pub.publish(self.last_command)
        else:
            # Landing mode - generate descent command
            if self.landing_start_position is None:
                self.get_logger().warn('Landing active but no start position - skipping')
                return
            
            # Create landing command (Mode 4: north pos, east pos, down vel, yaw)
            cmd = Command()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'NED'
            
            cmd.mode = Command.MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE
            cmd.cmd1 = 0.0
            cmd.cmd2 = 0.0
            cmd.cmd3 = 0.0
            cmd.cmd4 = 0.0

            # cmd.mode = Command.MODE_NPOS_EPOS_DVEL_YAW  # Mode 4
            # cmd.mode = Command.MODE_NPOS_EPOS_DPOS_YAW # Mode 0
            
            # # Hold horizontal position where landing started
            # cmd.cmd1 = float(self.landing_start_position['n'])  # North position
            # cmd.cmd2 = float(self.landing_start_position['e'])  # East position
            # # cmd.cmd3 = float(self.descent_rate)                 # Down velocity (positive = descend)
            # cmd.cmd3 = 0.0 # ground position
            # cmd.cmd4 = float(self.landing_start_yaw)            # Yaw

            # cmd.mode = Command.MODE_NACC_EACC_DACC_YAWRATE # Mode 10
            # cmd.cmd1 = 0.0
            # cmd.cmd2 = 0.0
            # cmd.cmd3 = 1.0
            # cmd.cmd4 = 0.0

            cmd.cmd_valid = True
            
            self.cmd_pub.publish(cmd)
            
            # Periodic status
            if self.current_uav_state:
                current_d = self.current_uav_state.p_d
                altitude_agl = -current_d  # Approximate altitude
                
                if int(self.get_clock().now().nanoseconds / 1e9) % 2 == 0:
                    self.get_logger().info(
                        f'Landing: Alt≈{altitude_agl:.2f}m, '
                        f'Descending at {self.descent_rate} m/s',
                        throttle_duration_sec=2.0
                    )

def main():
    rclpy.init()
    node = LandingCommander()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()