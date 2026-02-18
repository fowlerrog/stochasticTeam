#!/usr/bin/env python3
"""
plan_manager.py - Loads an offline plan, executes it by sending waypoints to UAV and UGV, and optionally also calls an online planner periodically
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from roscopter_msgs.srv import AddWaypoint
from roscopter_msgs.msg import Waypoint, State
from rosflight_msgs.msg import Status
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
import yaml
from enum import Enum
from scipy.spatial.distance import euclidean
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType

class ExecutionStates(Enum):
    START = 0
    MOVING_TOGETHER_TO_RELEASE = 1
    PREPARING_UAV = 2
    WAITING_FOR_TAKEOFF = 3
    MOVING_APART = 4
    WAITING_FOR_LANDING = 5
    FINISHED = 6
    WAITING_FOR_REPLAN_TOGETHER = 7
    WAITING_FOR_REPLAN_APART = 8

class PlanManager(Node):
    def __init__(self):
        super().__init__('plan_manager')

        self.get_logger().info('Plan Manager started')

        # Declare parameters
        self.declare_parameter('plan_filepath', '')
        self.declare_parameter('uav_waypoint_add_service', '/path_planner/add_waypoint')
        self.declare_parameter('uav_waypoint_clear_service', '/path_planner/clear_waypoints')
        self.declare_parameter('uav_odom_topic', '/estimated_state')
        self.declare_parameter('uav_status_topic', '/status')
        self.declare_parameter('uav_arm_service', '/toggle_arm')
        self.declare_parameter('uav_override_service', '/toggle_override')
        self.declare_parameter('ugv_waypoint_add_topic', '/ugv/waypoint')
        self.declare_parameter('ugv_waypoint_clear_service', '/ugv/waypoint_clear')
        self.declare_parameter('ugv_odom_topic', '/ugv/odom')
        self.declare_parameter('path_manager_set_params_service', '/path_manager/set_parameters')
        self.declare_parameter('distance_tolerance', 1.0)

        # Get parameters
        self.plan_filepath = self.get_parameter('plan_filepath').value
        self.uav_waypoint_add_service = self.get_parameter('uav_waypoint_add_service').value
        self.uav_waypoint_clear_service = self.get_parameter('uav_waypoint_clear_service').value
        self.uav_odom_topic = self.get_parameter('uav_odom_topic').value
        self.uav_status_topic = self.get_parameter('uav_status_topic').value
        self.uav_arm_service = self.get_parameter('uav_arm_service').value
        self.uav_override_service = self.get_parameter('uav_override_service').value
        self.ugv_waypoint_add_topic = self.get_parameter('ugv_waypoint_add_topic').value
        self.ugv_waypoint_clear_service = self.get_parameter('ugv_waypoint_clear_service').value
        self.ugv_odom_topic = self.get_parameter('ugv_odom_topic').value
        self.path_manager_set_params_service = self.get_parameter('path_manager_set_params_service').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value

        # Publishers and subscribers
        self.uav_waypoint_add_cli = self.create_client(AddWaypoint, self.uav_waypoint_add_service)
        self.uav_waypoint_clear_cli = self.create_client(Trigger, self.uav_waypoint_clear_service)
        self.uav_odom_sub = self.create_subscription(State, self.uav_odom_topic, self.uav_odom_callback, 10)
        self.uav_status_sub = self.create_subscription(Status, self.uav_status_topic, self.uav_status_callback, 10)
        self.uav_arm_cli = self.create_client(Trigger, self.uav_arm_service)
        self.uav_override_cli = self.create_client(Trigger, self.uav_override_service)
        self.ugv_waypoint_add_pub = self.create_publisher(PoseStamped, self.ugv_waypoint_add_topic, 10)
        self.ugv_waypoint_clear_cli = self.create_client(Trigger, self.ugv_waypoint_clear_service)
        self.ugv_odom_sub = self.create_subscription(Odometry, self.ugv_odom_topic, self.ugv_odom_callback, 10)
        self.path_manager_set_params_cli = self.create_client(SetParameters, self.path_manager_set_params_service)
        self.get_logger().info('Set up all publishers and subscribers')

        # Load plan
        self.params = {}
        if len(self.plan_filepath) > 0:
            self.get_logger().info(f'Loading plan from {self.plan_filepath}')
            self.load_plan(self.plan_filepath)

        # If we have a valid plan, initialize execution
        self.uav_position = None
        self.ugv_position = None
        self.i_tour = 0
        self.j_tour = 0
        self.uav_status = None
        self.pending_futures = {} # {'name' : future} callbacks we are waiting on
        self.service_error = False # any service has returned an error

        if len(self.params) == 0 or \
        'uav_tours' not in self.params or \
        len(self.params['uav_tours']) == 0 or \
        'uav_points' not in self.params or \
        len(self.params['uav_points']) == 0 or \
        'ugv_path' not in self.params or \
        len(self.params['ugv_path']) == 0 or \
        'ugv_point_map' not in self.params or \
        len(self.params['ugv_point_map']) == 0:
            self.get_logger().info(f'Nothing to do')
            self.state = ExecutionStates.FINISHED
        else:
            self.get_logger().info(f'Beginning plan execution')
            self.state = ExecutionStates.START

        # Timer for control loop
        self.timer = self.create_timer(0.5, self.control_loop)

    def control_loop(self):
        # Main control loop

        # Invalid states
        if self.state == ExecutionStates.FINISHED or \
            self.uav_position is None or \
            self.ugv_position is None:
            return

        # Check for and remove returned service calls
        completed = []
        for name, future in self.pending_futures.items():
            if future.done():
                completed.append(name)
                self.handle_service_response(name, future.result())
        for name in completed:
            del self.pending_futures[name]

        # Do not bother with state machine while we're waiting for returns or have an error
        if len(self.pending_futures) > 0 or self.service_error:
            return

        uav_tours = self.params['uav_tours']
        uav_points = self.params['uav_points']
        ugv_path = self.params['ugv_path']
        ugv_points = self.params['ugv_point_map']

        # Enter state machine
        if self.state == ExecutionStates.START:
            ugv_path = self.params['ugv_path']
            ugv_points = self.params['ugv_point_map']
            ugv_goal = ugv_points[ugv_path[2*self.i_tour + 1]]
            self.send_ugv_waypoint(ugv_goal)
            self.state = ExecutionStates.MOVING_TOGETHER_TO_RELEASE
            self.get_logger().info(f'Entering MOVING_TOGETHER_TO_RELEASE @ iTour = {self.i_tour}')
            self.set_hold_last()

        elif self.state == ExecutionStates.MOVING_TOGETHER_TO_RELEASE:
            ugv_goal = ugv_points[ugv_path[2*self.i_tour + 1]]
            ugv_at_position = self.get_agent_distance(ugv_goal, 'UGV') < self.distance_tolerance

            if ugv_at_position and self.uav_status is not None:
                # check for final point
                if self.i_tour == len(uav_tours):
                    self.state = ExecutionStates.FINISHED
                    self.get_logger().info(f"Entering FINISHED")
                    return

                # prepare for takeoff
                self.toggle_uav_arm(True)
                self.clear_uav_waypoints()
                self.state = ExecutionStates.PREPARING_UAV
                self.get_logger().info(f"Entering PREPARING_UAV @ iTour = {self.i_tour}")

        elif self.state == ExecutionStates.PREPARING_UAV:
            # command takeoff
            self.send_uav_waypoint(uav_points[uav_tours[self.i_tour][0]])
            self.toggle_uav_override(False)
            self.state = ExecutionStates.WAITING_FOR_TAKEOFF
            self.get_logger().info(f"Entering WAITING_FOR_TAKEOFF @ iTour = {self.i_tour}")

        elif self.state == ExecutionStates.WAITING_FOR_TAKEOFF:
            uav_goal = uav_points[uav_tours[self.i_tour][0]]
            uav_at_position = self.get_agent_distance(uav_goal, 'UAV') < self.distance_tolerance

            if uav_at_position:
                # takeoff complete
                self.send_ugv_waypoint(ugv_points[ugv_path[2*self.i_tour + 2]])
                self.state = ExecutionStates.MOVING_APART
                self.get_logger().info(f"Entering MOVING_APART @ iTour = {self.i_tour}")

        elif self.state == ExecutionStates.MOVING_APART:
            uav_goal = uav_points[uav_tours[self.i_tour][self.j_tour]]
            uav_at_position = self.get_agent_distance(uav_goal, 'UAV') < self.distance_tolerance

            if uav_at_position:
                # check for next point
                if self.j_tour < len(uav_tours[self.i_tour]) - 1:
                    self.send_uav_waypoint(uav_points[uav_tours[self.i_tour][self.j_tour + 1]])
                    self.j_tour += 1

                # otherwise, last point and check for ugv position
                ugv_goal = ugv_points[ugv_path[2*self.i_tour + 2]]
                ugv_at_position = self.get_agent_distance(ugv_goal, 'UGV') < self.distance_tolerance
                if ugv_at_position:
                    self.send_uav_waypoint([*ugv_goal[:2], 0]) #TODO vertical offset
                    self.state = ExecutionStates.WAITING_FOR_LANDING
                    self.get_logger().info(f"Entering WAITING_FOR_LANDING @ iTour = {self.i_tour}")

        elif self.state == ExecutionStates.WAITING_FOR_LANDING:
            uav_goal = uav_points[uav_tours[self.i_tour][-1]]
            uav_at_position = self.get_agent_distance(uav_goal, 'UAV') < self.distance_tolerance

            if uav_at_position:
                # landing complete
                self.toggle_uav_override(True)
                self.i_tour += 1
                self.j_tour = 0

                ugv_path = self.params['ugv_path']
                ugv_points = self.params['ugv_point_map']
                ugv_goal = ugv_points[ugv_path[2*self.i_tour + 1]]
                self.send_ugv_waypoint(ugv_goal)

                self.state = ExecutionStates.MOVING_TOGETHER_TO_RELEASE
                self.get_logger().info(f"Entering MOVING_TOGETHER_TO_RELEASE @ iTour = {self.i_tour}")

    def replanner_callback(self, msg):
        # Receive updated plan
        if self.state == ExecutionStates.WAITING_FOR_REPLAN_APART:
            self.state = ExecutionStates.MOVING_APART
        elif self.state == ExecutionStates.WAITING_FOR_REPLAN_TOGETHER:
            self.state = ExecutionStates.MOVING_TOGETHER_TO_RELEASE
        pass

    def uav_status_callback(self, msg):
        # Update UAV status
        self.uav_status = msg

    def uav_odom_callback(self, msg):
        # Update UAV position
        self.uav_position = [msg.p_n, -msg.p_e, -msg.p_d] # NED -> world

    def ugv_odom_callback(self, msg):
        # Update UGV position
        self.ugv_position = msg.pose.pose.position

    def get_agent_distance(self, goal, agentType):
        if agentType == 'UAV':
            if self.uav_position is None:
                return None
            # self.get_logger().info(f'UAV distance {euclidean(goal, self.uav_position)} b/t {goal}, {self.uav_position}')
            return euclidean(goal, self.uav_position)
        elif agentType == 'UGV':
            if self.ugv_position is None:
                return None
            return euclidean(goal, [self.ugv_position.x, self.ugv_position.y])
        self.get_logger().error(f'get_agent_distance: agentType {agentType} not recognized')
        return None

    def load_plan(self, plan_filepath):
        try:
            with open(plan_filepath, 'r') as f:
                self.params = yaml.safe_load(f)
        except Exception:
            node.get_logger().error(f"Failed to read yaml file {plan_filepath}")

    def handle_service_response(self, name, response):
        # Handle service responses
        if response is not None and \
        (not hasattr(response, 'success') or response.success):
            self.get_logger().info(f'{name} succeeded: {response.message if hasattr(response, 'message') else '(NO MESSAGE)'}')
        else:
            self.get_logger().error(f'{name} failed')
            self.service_error = True

    def call_service(self, name, client, request):
        # Wait for client to be available
        self.get_logger().info(f'Waiting for service {name}...')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {name} not available, waiting...')
        self.get_logger().info(f'Service {name} available!')

        # Call service
        self.pending_futures[name] = client.call_async(request)

    def set_hold_last(self):
        # Tell the /path_manager to hold the last UAV waypoint instead of looping the list
        request = SetParameters.Request()
        param = Parameter()
        param.name = 'hold_last'
        param.value.type = ParameterType.PARAMETER_BOOL
        param.value.bool_value = True
        request.parameters.append(param)
        self.get_logger().info(f'Setting /path_manager.hold_last := True')
        self.call_service(self.path_manager_set_params_service, self.path_manager_set_params_cli, request)

    def toggle_uav_arm(self, value=True):
        # Toggle UAV motor arm if necessary
        if self.uav_status.armed != value:
            self.get_logger().info(f'Setting UAV arm to {value}')
            self.call_service(self.uav_arm_service, self.uav_arm_cli, Trigger.Request())

    def toggle_uav_override(self, value=False):
        # Toggle UAV control override if necessary
        if (self.uav_status.control_mode > 0) != value:
            self.get_logger().info(f'Setting UAV override to {value}')
            self.call_service(self.uav_override_service, self.uav_override_cli, Trigger.Request())

    def clear_uav_waypoints(self):
        # Clear existing UAV waypoints
        self.get_logger().info(f'Clearing UAV waypoints')
        self.call_service(self.uav_waypoint_clear_service, self.uav_waypoint_clear_cli, Trigger.Request())

    def send_uav_waypoint(self, position):
        # Send UAV waypoint service
        request = AddWaypoint.Request()
        request.wp.w = [position[0], -position[1], -position[2]] # world -> NED
        self.get_logger().info(f'Sending UAV waypoint [NED] {request.wp.w}')
        self.call_service(self.uav_waypoint_add_service, self.uav_waypoint_add_cli, request)

    def clear_ugv_waypoints(self):
        # Clear existing UGV waypoints
        self.get_logger().info(f'Clearing UGV waypoints')
        self.call_service(self.ugv_waypoint_clear_service, self.ugv_waypoint_clear_cli, Trigger.Request())

    def send_ugv_waypoint(self, position):
        # Construct a UGV waypoint message
        msg = PoseStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        
        # Wait for subscriber
        self.get_logger().info(f'Waiting for subscriber on {self.ugv_waypoint_add_topic}...')
        while self.ugv_waypoint_add_pub.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.ugv_waypoint_add_pub.publish(msg)
        self.get_logger().info(f"Sent waypoint: {position} to {self.ugv_waypoint_add_topic}")

        return True


def main(args=None):
    rclpy.init(args=args)
    node = PlanManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
