#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosflight_msgs.msg import GNSS
from roscopter_msgs.msg import State, ControllerCommand

class DiagnosticLogger(Node):
    def __init__(self):
        super().__init__('extended_diagnostic_logger')
        
        self.create_subscription(GNSS, '/gnss', self.gnss_callback, 10)
        self.create_subscription(State, '/estimated_state', self.state_callback, 10)
        self.create_subscription(State, '/sim/roscopter/state', self.truth_callback, 10)
        
        # Add controller command monitoring
        self.create_subscription(
            ControllerCommand, 
            '/high_level_command', 
            self.cmd_callback, 
            10
        )
        
        self.gnss_msg = None
        self.est_msg = None
        self.truth_msg = None
        self.cmd_msg = None
        
        self.create_timer(1.0, self.log_status)
    
    def gnss_callback(self, msg):
        self.gnss_msg = msg
    
    def state_callback(self, msg):
        self.est_msg = msg
    
    def truth_callback(self, msg):
        self.truth_msg = msg
    
    def cmd_callback(self, msg):
        self.cmd_msg = msg
        # Log immediately when command changes
        self.get_logger().info(f'CMD: mode={msg.mode}, cmd3={msg.cmd3:.3f} (altitude/down)')
    
    def log_status(self):
        if self.gnss_msg and self.est_msg and self.truth_msg:
            self.get_logger().info('='*50)
            self.get_logger().info(f'GNSS alt: {self.gnss_msg.alt:.3f}')
            self.get_logger().info(f'Truth p_d: {self.truth_msg.p_d:.3f}')
            self.get_logger().info(f'Est p_d: {self.est_msg.p_d:.3f}')
            self.get_logger().info(f'Truth v_z: {self.truth_msg.v_z:.3f}')
            self.get_logger().info(f'Est v_z: {self.est_msg.v_z:.3f}')
            
            if self.cmd_msg:
                self.get_logger().info(f'Commanded mode: {self.cmd_msg.mode}, cmd3: {self.cmd_msg.cmd3:.3f}')
            else:
                self.get_logger().warn('NO COMMAND BEING SENT!')

def main():
    rclpy.init()
    node = DiagnosticLogger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()