#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import sys
import yaml
from roscopter_msgs.srv import AddWaypoint
from roscopter_msgs.msg import Waypoint

def main():
    if len(sys.argv) < 2:
        print("Usage: ros2 run uav_ugv_teaming load_plan /path/to/plan_path_results.yaml")
        return

    rclpy.init()
    node = Node('plan_loader')

    # Load yaml file
    filename = sys.argv[1]
    try:
        with open(filename, 'r') as f:
            params = yaml.safe_load(f)
    except Exception:
        node.get_logger().error(f"Failed to real yaml file {filename}")
        rclpy.shutdown()
        return

    # Send UGV points
    ugvOrder = params['ugv_path']
    ugvPoints = params['ugv_point_map']

    poses = []
    for n in ugvOrder:
        x, y = ugvPoints[n][:2]
        msg = PoseStamped()
        msg.pose.position.x = x
        msg.pose.position.y = y
        # msg.pose.position.z = 0.0
        poses.append(msg)
    ugvMessage = Path()
    ugvMessage.header.frame_id = 'world'
    ugvMessage.header.stamp = node.get_clock().now().to_msg()
    ugvMessage.poses = poses

    ugvTopic = '/ugv/waypoint_path'
    ugvPub = node.create_publisher(Path, ugvTopic, 10)

    # Wait for subscriber
    node.get_logger().info(f'Waiting for subscriber on {ugvTopic}...')
    while ugvPub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)

    ugvPub.publish(ugvMessage)
    node.get_logger().info(f"Sent path of {len(poses)} points to {ugvTopic}")

    # Send UAV points
    uavOrder = []
    for tour in params['uav_tours']:
        uavOrder.extend(tour)
    uavPoints = params['uav_points']

    waypointAdder = WaypointAdder()
    for n in uavOrder:
        x, y, z = uavPoints[n]
        waypointAdder.add_waypoint(x, y, -z)
    node.get_logger().info(f"Sent path of {len(uavOrder)} points to /path_planner/add_waypoint service")

    rclpy.shutdown()

class WaypointAdder(Node):
    def __init__(self):
        super().__init__('waypoint_adder')
        
        # Create service client
        self.client = self.create_client(AddWaypoint, '/path_planner/add_waypoint')
        
        # Wait for service to be available
        self.get_logger().info('Waiting for /path_planner/add_waypoint service...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.get_logger().info('Service available!')
    
    def add_waypoint(self, n, e, d):
        """Add a waypoint via service call"""
        request = AddWaypoint.Request()
        
        # Fill in the request - adjust fields based on actual service definition
        request.wp.w = [n, e, d]
        
        # Call service
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            self.get_logger().info(f'Waypoint added successfully')
            return True
        else:
            self.get_logger().error('Service call failed')
            return False

if __name__ == '__main__':
    main()
