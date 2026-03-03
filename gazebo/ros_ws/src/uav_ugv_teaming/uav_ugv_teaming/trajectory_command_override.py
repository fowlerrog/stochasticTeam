#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from roscopter_msgs.msg import TrajectoryCommand
from roscopter_msgs.srv import AddWaypoint
from scipy.spatial.distance import euclidean
from std_srvs.srv import Trigger

class TrajectoryCommandOverride(Node):
    def __init__(self):
        super().__init__('trajectory_command_override')
        
        # Declare and evaluate parameters
        paramList = [
            ('raw_command_topic', '/trajectory_command'),
            ('command_topic', '/trajectory_command_override'),
            ('set_goal_service', '/trajectory_command_override/set_goal'),
            ('clear_integrators_service', '/trajectory_follower/clear_integrators'),
            ('fine_distance_tolerance', 0.2)
        ]
        for name, defaultValue in paramList:
            self.declare_parameter(name, defaultValue)
            setattr(self, name, self.get_parameter(name).value)

        # State
        self.active = False
        self.waiting_for_integrator_clear = False
        self.goal_position = None
        self.last_command = None
        self.pending_future = None

        # Subscriber
        self.cmd_sub = self.create_subscription(
            TrajectoryCommand,
            self.raw_command_topic,
            self.cmd_callback,
            10
        )        
        
        # Publisher
        self.cmd_pub = self.create_publisher(
            TrajectoryCommand,
            self.command_topic,
            10
        )
        
        # Client to clear trajectory follower PID integrations
        self.clear_integrators_cli = self.create_client(
            Trigger,
            self.clear_integrators_service
        )

        # Service to set goal
        self.set_goal_srv = self.create_service(
            AddWaypoint,
            self.set_goal_service,
            self.set_goal_service_callback
        )

        # Timer to publish at consistent rate
        self.timer = self.create_timer(0.0025, self.publish_command)  # 400 Hz

        self.get_logger().info('Trajectory Command Override started')
    
    def cmd_callback(self, msg):
        """Store the latest command"""
        self.last_command = msg
    
    def set_goal_service_callback(self, request, response):
        """Set goal and activate via service"""
        if request:
            # Activate goal
            self.goal_position = request.wp.w
            self.active = True

            response.success = True
            response.message = (
                f'Trajectory command override activated @ {self.goal_position}'
            )
            self.get_logger().info(response.message)
            
        else:
            self.active = False # deactivate
            response.success = False
            response.message = 'Received bad goal, not activating'
            self.get_logger().info(response.message)
        
        return response
    
    def publish_command(self):
        """Publish command - either passthrough or position override"""

        # the only return we should get is for clearing integrators
        if self.pending_future and self.pending_future.done():
            self.waiting_for_integrator_clear = False
            self.pending_futures = {}

        # until we start receiving trajectory commands that match the goal position, send our own
        if not self.active and not self.waiting_for_integrator_clear:
            if self.last_command is not None: # pass through
                self.cmd_pub.publish(self.last_command)

        else:
            cmd = TrajectoryCommand()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = ''
            cmd.position = self.goal_position
            self.cmd_pub.publish(cmd)

            # check for deactivation
            if self.active and \
            self.last_command is not None and \
            euclidean(self.goal_position, self.last_command.position) < self.fine_distance_tolerance:
                self.active = False
                self.waiting_for_integrator_clear = True
                self.get_logger().info('Received trajectory command near goal, clearing trajectory integrations')
                # Also clear trajectory follower PID integration, and wait for that to trigger before actually deactivating
                self.call_clear_integrators()

    def call_clear_integrators(self):
        while not self.clear_integrators_cli.wait_for_service(timeout_sec=1.0):
            pass
        self.pending_future = self.clear_integrators_cli.call_async(Trigger.Request())

def main():
    rclpy.init()
    node = TrajectoryCommandOverride()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()