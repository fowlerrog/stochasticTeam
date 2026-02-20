#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from roscopter_msgs.msg import ControllerCommand, State

class CommandDebugger(Node):
    def __init__(self):
        super().__init__('command_debugger')
        
        self.create_subscription(ControllerCommand, '/high_level_command_override',
                                self.cmd_callback, 10)
        self.create_subscription(State, '/estimated_state_override',
                                self.state_callback, 10)
        
        self.cmd = None
        self.state = None
        self.create_timer(1.0, self.debug)
    
    def cmd_callback(self, msg):
        self.cmd = msg
    
    def state_callback(self, msg):
        self.state = msg
    
    def debug(self):
        if self.cmd and self.state:
            self.get_logger().info('='*60)
            self.get_logger().info(f'COMMAND:')
            self.get_logger().info(f'  Mode: {self.cmd.mode}')
            self.get_logger().info(f'  cmd1 (N pos): {self.cmd.cmd1:.3f}')
            self.get_logger().info(f'  cmd2 (E pos): {self.cmd.cmd2:.3f}')
            self.get_logger().info(f'  cmd3 (D vel): {self.cmd.cmd3:.3f}')
            self.get_logger().info(f'  cmd4 (Yaw): {self.cmd.cmd4:.3f}')
            self.get_logger().info(f'STATE:')
            self.get_logger().info(f'  p_d: {self.state.p_d:.3f} (down position)')
            self.get_logger().info(f'  v_z: {self.state.v_z:.3f} (down velocity)')
            self.get_logger().info(f'  Altitude: {-self.state.p_d:.3f}m')

def main():
    rclpy.init()
    node = CommandDebugger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()