#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from uav_ugv_teaming_msgs.srv import CallScript  # our custom service
import subprocess
import json

class VenvScriptCaller(Node):
    def __init__(self):
        super().__init__('venv_script_caller')
        
        # Declare and evaluate parameters
        paramList = [
            ('venv_path', '/path/to/your/venv'),
            ('script_path', '/path/to/your/script.py'),
            ('default_timeout', 60.0)
        ]
        for name, defaultValue in paramList:
            self.declare_parameter(name, defaultValue)
            setattr(self, name, self.get_parameter(name).value)
        
        self.venv_python = f'{self.venv_path}/bin/python'
        
        # Service with arguments
        self.call_srv = self.create_service(
            CallScript,
            'call_venv_script',
            self.call_script_service
        )
        
        self.get_logger().info('VEnv Script Caller started')
        self.get_logger().info(f'  Service: /call_venv_script (CallScript)')
    
    def call_script_service(self, request, response):
        """Service callback - calls script with provided arguments"""
        
        # Get timeout (use default if not specified)
        timeout = request.timeout if request.timeout > 0 else self.default_timeout
        
        # Build command with arguments
        cmd = [self.venv_python, self.script_path] + list(request.args)
        
        # Prepare stdin if JSON provided
        stdin_input = request.input_json if request.input_json else None
        
        try:
            self.get_logger().info(f'Calling: {" ".join(cmd)}')
            if stdin_input:
                self.get_logger().info(f'With input: {stdin_input[:100]}...')
            
            # Run script (BLOCKING)
            result = subprocess.run(
                cmd,
                input=stdin_input,
                capture_output=True,
                text=True,
                timeout=timeout,
                check=False
            )
            
            # Fill response
            response.success = (result.returncode == 0)
            response.stdout = result.stdout
            response.stderr = result.stderr
            response.returncode = result.returncode
            
            if response.success:
                self.get_logger().info(f'Script succeeded: {result.stdout[:100]}')
            else:
                self.get_logger().error(f'Script failed (code {result.returncode}): {result.stderr[:100]}')
            
        except subprocess.TimeoutExpired:
            self.get_logger().error(f'Script timed out after {timeout}s')
            response.success = False
            response.stdout = ''
            response.stderr = f'Timeout after {timeout}s'
            response.returncode = -1
            
        except Exception as e:
            self.get_logger().error(f'Exception: {e}')
            response.success = False
            response.stdout = ''
            response.stderr = str(e)
            response.returncode = -1
        
        return response

def main():
    rclpy.init()
    node = VenvScriptCaller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()