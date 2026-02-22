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
from std_srvs.srv import Trigger, SetBool
from nav_msgs.msg import Odometry, Path
import yaml
from enum import Enum
from scipy.spatial.distance import euclidean
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType
from std_msgs.msg import Bool
from uav_ugv_teaming_msgs.srv import CallScript
import json

class ExecutionStates(Enum):
    VALID_PLAN = -6 # valid settings but no action
    PRE_READYING = -5 # UAV being prepared for flight
    READYING_HORIZONTAL = -4 # UAV moving to UGV
    READYING_LANDING = -3 # UAV landing on UGV
    READYING_FINAL = -2 # UAV landed
    READY = -1 # wait for command
    START = 0 # begin plan
    WAITING_FOR_REPLAN_TOGETHER = 1
    MOVING_TOGETHER_TO_RELEASE = 2
    SENDING_UAV_WAYPOINT = 3
    ENABLING_UAV = 4
    WAITING_FOR_TAKEOFF = 5
    WAITING_FOR_REPLAN_APART = 6
    MOVING_APART = 7
    WAITING_FOR_LANDING_HORIZONTAL = 8 # UAV moving to UGV
    WAITING_FOR_LANDING_VERTICAL = 9 # UAV landing on UGV
    FINISHED = 10

class PlanManager(Node):
    def __init__(self):
        super().__init__('plan_manager')

        self.get_logger().info('Plan Manager started')

        # Declare and evaluate parameters
        paramList = [
            ('plan_filepath', ''),
            ('online_planner_filepath', ''),
            ('uav_waypoint_add_service', '/path_planner/add_waypoint'),
            ('uav_waypoint_clear_service_1', '/path_planner/clear_waypoints'),
            ('uav_waypoint_clear_service_2', '/path_manager/clear_waypoints'),
            ('traj_override_service', '/trajectory_command_override/set_goal'),
            ('uav_odom_topic', '/estimated_state'),
            ('uav_status_topic', '/status'),
            ('uav_arm_service', '/toggle_arm'),
            ('uav_override_service', '/toggle_override'),
            ('ugv_waypoint_path_topic', '/ugv/waypoint_path'),
            ('ugv_waypoint_clear_service', '/ugv/waypoint_clear'),
            ('ugv_odom_topic', '/ugv/odom'),
            ('path_manager_set_params_service', '/path_manager/set_parameters'),
            ('uav_ugv_in_contact_topic', '/collision_force_injector/in_contact'),
            ('coarse_distance_tolerance', 0.7), # for UAV waypoints, for example
            ('fine_distance_tolerance', 0.1), # for UGV waypoints
            ('fine_velocity_tolerance', 0.1),
            ('ugv_landing_height', 0.5), # UGV landing offset
            ('activate_landing_service', '/landing_commader/toggle'),
            ('activate_takeoff_service', '/takeoff_commader/toggle'),
            ('wait_time', 2.0),
            ('online_planner_service', '/call_venv_script'),
        ]
        for name, defaultValue in paramList:
            self.declare_parameter(name, defaultValue)
            setattr(self, name, self.get_parameter(name).value)

        # Publishers and subscribers
        self.uav_waypoint_add_cli = self.create_client(AddWaypoint, self.uav_waypoint_add_service)
        self.uav_waypoint_clear_cli_1 = self.create_client(Trigger, self.uav_waypoint_clear_service_1)
        self.uav_waypoint_clear_cli_2 = self.create_client(Trigger, self.uav_waypoint_clear_service_2)
        self.traj_override_cli = self.create_client(AddWaypoint, self.traj_override_service)
        self.uav_odom_sub = self.create_subscription(State, self.uav_odom_topic, self.uav_odom_callback, 10)
        self.uav_status_sub = self.create_subscription(Status, self.uav_status_topic, self.uav_status_callback, 10)
        self.uav_arm_cli = self.create_client(Trigger, self.uav_arm_service)
        self.uav_override_cli = self.create_client(Trigger, self.uav_override_service)
        self.ugv_waypoint_path_pub = self.create_publisher(Path, self.ugv_waypoint_path_topic, 10)
        self.ugv_waypoint_clear_cli = self.create_client(Trigger, self.ugv_waypoint_clear_service)
        self.ugv_odom_sub = self.create_subscription(Odometry, self.ugv_odom_topic, self.ugv_odom_callback, 10)
        self.path_manager_set_params_cli = self.create_client(SetParameters, self.path_manager_set_params_service)
        self.uav_ugv_in_contact_sub = self.create_subscription(Bool, self.uav_ugv_in_contact_topic, self.uav_ugv_in_contact_callback, 10)
        self.activate_landing_cli = self.create_client(SetBool, self.activate_landing_service)
        self.activate_takeoff_cli = self.create_client(SetBool, self.activate_takeoff_service)

        # Subscribe to start service
        self.start_service = self.create_service(
            Trigger,
            '/plan_manager/start',
            self.start_callback
        )

        self.get_logger().info('Set up all publishers and subscribers')

        # Load plan
        self.plan_params = {}
        if len(self.plan_filepath) > 0:
            self.get_logger().info(f'Loading plan from {self.plan_filepath}')
            self.plan_params = self.load_yaml(self.plan_filepath)

        # Set up online planner
        self.online_planner_params = {}
        if len(self.online_planner_filepath) > 0:
            self.get_logger().info(f'Loading online planner parameters from {self.online_planner_filepath}')
            self.online_planner_params = self.load_yaml(self.online_planner_filepath)
            self.online_planner_cli = self.create_client(CallScript, 'call_venv_script')

        # If we have a valid plan, initialize execution
        self.uav_position = None
        self.uav_velocity = None
        self.uav_goal = None
        self.uav_takeoff_time = None

        self.ugv_position = None
        self.ugv_velocity = None
        self.ugv_goal = None

        self.i_tour = 0
        self.j_tour = 0

        self.uav_status = None
        self.in_contact = False

        self.pending_futures = {} # {'name' : future} callbacks we are waiting on
        self.service_error = False # any service has returned an error

        if len(self.plan_params) == 0 or \
        'uav_tours' not in self.plan_params or \
        len(self.plan_params['uav_tours']) == 0 or \
        'uav_points' not in self.plan_params or \
        len(self.plan_params['uav_points']) == 0 or \
        'ugv_path' not in self.plan_params or \
        len(self.plan_params['ugv_path']) == 0 or \
        'ugv_point_map' not in self.plan_params or \
        len(self.plan_params['ugv_point_map']) == 0:
            self.get_logger().info(f'Nothing to do')
            self.state = ExecutionStates.FINISHED
        else:
            self.get_logger().info(f'Entering VALID_PLAN')
            self.state = ExecutionStates.VALID_PLAN

        # Timer for mission waiting
        self.waiting_for_timer = False
        self.mission_timer = self.create_timer(self.wait_time, self.timer_callback, autostart=False)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        # Main control loop

        # Invalid states
        if self.state == ExecutionStates.FINISHED or \
            self.state == ExecutionStates.READY or \
            self.uav_position is None or \
            self.ugv_position is None or \
            self.waiting_for_timer:
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

        uav_tours = self.plan_params['uav_tours']
        uav_points = self.plan_params['uav_points']
        ugv_path = self.plan_params['ugv_path']
        ugv_points = self.plan_params['ugv_point_map']

        # Enter state machine
        if self.state == ExecutionStates.PRE_READYING:
            # Prepare UAV for first flight
            self.set_hold_last()
            self.toggle_uav_arm(True)
            self.send_ugv_waypoint([0.0,0.0]) # shake the UGV a bit so it doesn't buck the UAV when it starts up
            self.state = ExecutionStates.READYING_HORIZONTAL
            self.get_logger().info(f"Entering READYING_HORIZONTAL")

        elif self.state == ExecutionStates.READYING_HORIZONTAL:
            # Command initial hover
            self.uav_goal = [a + b for a,b in zip([self.ugv_position.x, self.ugv_position.y, self.ugv_position.z], [0, 0, 2*self.ugv_landing_height])]
            self.send_uav_waypoint(self.uav_goal)
            self.toggle_uav_override(False)
            self.state = ExecutionStates.READYING_LANDING
            self.get_logger().info(f'Entering READYING_LANDING')

        elif self.state == ExecutionStates.READYING_LANDING:
            if self.uav_at_goal():
                # Command initial landing
                self.command_landing(True)
                self.state = ExecutionStates.READYING_FINAL
                self.get_logger().info(f'Entering READYING_FINAL')

        elif self.state == ExecutionStates.READYING_FINAL:
            if self.in_contact and self.uav_status.control_mode == 1: # control disabled
                self.clear_uav_waypoints()
                self.state = ExecutionStates.READY
                self.get_logger().info(f'Entering READY')

        elif self.state == ExecutionStates.START:
            # Check for replan
            if len(self.online_planner_params) > 0:
                self.call_online_planner(True) # this updates the plan
            self.state = ExecutionStates.WAITING_FOR_REPLAN_TOGETHER
            self.get_logger().info(f'Entering WAITING_FOR_REPLAN_TOGETHER @ iTour = {self.i_tour}')

        elif self.state == ExecutionStates.WAITING_FOR_REPLAN_TOGETHER:
            # Activates as soon as we're not waiting for a replan
            ugv_path = self.plan_params['ugv_path']
            ugv_points = self.plan_params['ugv_point_map']
            self.ugv_goal = ugv_points[ugv_path[2*self.i_tour + 1]]
            self.send_ugv_waypoint(self.ugv_goal)
            self.state = ExecutionStates.MOVING_TOGETHER_TO_RELEASE
            self.get_logger().info(f'Entering MOVING_TOGETHER_TO_RELEASE @ iTour = {self.i_tour}')

        elif self.state == ExecutionStates.MOVING_TOGETHER_TO_RELEASE:
            if self.ugv_at_goal():
                # check for final UGV point
                if self.i_tour == len(uav_tours):
                    self.state = ExecutionStates.FINISHED
                    self.get_logger().info(f"Entering FINISHED")
                    return

                # prepare for takeoff
                self.clear_uav_waypoints()
                self.state = ExecutionStates.SENDING_UAV_WAYPOINT
                self.get_logger().info(f"Entering SENDING_UAV_WAYPOINT @ iTour = {self.i_tour}")

        elif self.state == ExecutionStates.SENDING_UAV_WAYPOINT:
            # wait for UAV waypoints to clear before sending
            self.uav_goal = uav_points[uav_tours[self.i_tour][0]]
            self.send_uav_waypoint(self.uav_goal)
            self.activate_trajectory_override(self.uav_goal)
            self.state = ExecutionStates.ENABLING_UAV
            self.get_logger().info(f"Entering ENABLING_UAV @ iTour = {self.i_tour}")

        elif self.state == ExecutionStates.ENABLING_UAV:
            # command takeoff after UAV has received waypoint
            self.command_takeoff(True)
            self.state = ExecutionStates.WAITING_FOR_TAKEOFF
            self.get_logger().info(f"Entering WAITING_FOR_TAKEOFF @ iTour = {self.i_tour}")
            self.start_timer() # wait before moving UGV or replanning
            self.uav_takeoff_time = self.get_clock().now()

        elif self.state == ExecutionStates.WAITING_FOR_TAKEOFF:
            # takeoff complete
            if self.uav_at_goal(fine=False, checkVel=False):
                # Check for replan
                if len(self.online_planner_params) > 0:
                    self.call_online_planner(False) # this updates the plan
                self.state = ExecutionStates.WAITING_FOR_REPLAN_APART
                self.get_logger().info(f'Entering WAITING_FOR_REPLAN_APART @ iTour, jTour = {self.i_tour}, {self.j_tour}')
                self.j_tour += 1

        elif self.state == ExecutionStates.WAITING_FOR_REPLAN_APART:
            # Activates as soon as we're not waiting for a replan
            # Send or update UGV goal
            potential_ugv_goal = ugv_points[ugv_path[2*self.i_tour + 2]]
            if euclidean(self.ugv_goal, potential_ugv_goal) > self.fine_distance_tolerance:
                self.send_ugv_waypoint(potential_ugv_goal) # replace existing waypoint (blocking)
                self.ugv_goal = potential_ugv_goal

            # If this is the last UAV point in the tour, wait for the UGV to arrive
            if self.j_tour == len(uav_tours[self.i_tour]):
                if self.ugv_at_goal():
                    self.uav_goal = [a + b for a,b in zip([self.ugv_position.x, self.ugv_position.y, self.ugv_position.z], [0, 0, 2*self.ugv_landing_height])]
                    self.send_uav_waypoint(self.uav_goal)
                    self.state = ExecutionStates.WAITING_FOR_LANDING_HORIZONTAL
                    self.get_logger().info(f"Entering WAITING_FOR_LANDING_HORIZONTAL @ iTour = {self.i_tour}")
                    self.start_timer() # wait before dropping
            else: # Send next UAV point
                self.uav_goal = uav_points[uav_tours[self.i_tour][self.j_tour]]
                self.send_uav_waypoint(self.uav_goal)
                self.state = ExecutionStates.MOVING_APART
                self.get_logger().info(f"Entering MOVING_APART @ iTour, jTour = {self.i_tour}, {self.j_tour}")

        elif self.state == ExecutionStates.MOVING_APART:
            # uav is at goal
            if self.uav_at_goal(fine=False, checkVel=False):
                # Check for replan and increment j
                if len(self.online_planner_params) > 0 and \
                self.j_tour < len(uav_tours[self.i_tour]) - 1: # no replanning at last air point - we'll do a replan right after landing
                    self.call_online_planner(False) # this updates the plan
                self.state = ExecutionStates.WAITING_FOR_REPLAN_APART
                self.get_logger().info(f'Entering WAITING_FOR_REPLAN_APART @ iTour, jTour = {self.i_tour}, {self.j_tour}')
                self.j_tour += 1

        elif self.state == ExecutionStates.WAITING_FOR_LANDING_HORIZONTAL:
            # Lining up vertically for landing
            if self.uav_at_goal():
                self.command_landing(True)
                self.state = ExecutionStates.WAITING_FOR_LANDING_VERTICAL
                self.get_logger().info(f"Entering WAITING_FOR_LANDING_VERTICAL @ iTour = {self.i_tour}")
                self.start_timer() # wait before leaving with UAV

        elif self.state == ExecutionStates.WAITING_FOR_LANDING_VERTICAL:
            if self.in_contact and self.uav_status.control_mode == 1: # control disabled
                self.clear_uav_waypoints()
                self.uav_takeoff_time = None # reset flight time

                self.i_tour += 1
                self.j_tour = 0

                # Check for replan
                if len(self.online_planner_params) > 0:
                    self.call_online_planner(True) # this updates the plan
                self.state = ExecutionStates.WAITING_FOR_REPLAN_TOGETHER
                self.get_logger().info(f"Entering WAITING_FOR_REPLAN_TOGETHER @ iTour = {self.i_tour}")

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
        self.uav_velocity = [msg.v_x, msg.v_y, msg.v_z] # body frame

    def ugv_odom_callback(self, msg):
        # Update UGV position
        self.ugv_position = msg.pose.pose.position
        self.ugv_velocity = msg.twist.twist.linear

    def uav_ugv_in_contact_callback(self, msg):
        # Update contact status
        self.in_contact = msg.data

    def start_callback(self, request, response):
        # Process start request
        if self.state == ExecutionStates.VALID_PLAN:
            self.state = ExecutionStates.PRE_READYING
            msgString = 'Entering PRE_READYING'
        elif self.state == ExecutionStates.READY:
            if self.in_contact:
                self.state = ExecutionStates.START
                msgString = 'Entering START'
            else:
                msgString = 'Cannot enter START because UAV is not landed'
        else:
            msgString = 'Nothing to start'
        self.get_logger().info(msgString)
        response.success = True
        response.message = msgString
        return response

    def start_timer(self):
        self.waiting_for_timer = True
        self.mission_timer.reset()
        self.get_logger().info(f'Starting timer for {self.wait_time}s')

    def timer_callback(self):
        self.waiting_for_timer = False
        self.mission_timer.cancel()
        self.get_logger().info(f'Timer is up')

    def uav_at_goal(self, fine=True, checkVel=True):
        # is the UAV in xy radius and optionally stopped
        distance_tolerance = self.fine_distance_tolerance if fine else self.coarse_distance_tolerance
        return euclidean(self.uav_goal[:2], self.uav_position[:2]) < distance_tolerance and \
            (not checkVel or sum(v**2 for v in self.uav_velocity) < self.fine_velocity_tolerance**2)

    def ugv_at_goal(self, fine=True, checkVel=True):
        # is the UGV in xy radius and optionally stopped
        distance_tolerance = self.fine_distance_tolerance if fine else self.coarse_distance_tolerance
        return euclidean(self.ugv_goal[:2], [self.ugv_position.x, self.ugv_position.y]) < distance_tolerance and \
            (not checkVel or sum(v**2 for v in [self.ugv_velocity.x, self.ugv_velocity.y, self.ugv_velocity.z]) < self.fine_velocity_tolerance**2)

    def load_yaml(self, filepath):
        try:
            with open(filepath, 'r') as f:
                return yaml.safe_load(f)
        except Exception:
            node.get_logger().error(f"Failed to read yaml file {filepath}")

    def handle_service_response(self, name, response):
        # Handle service responses
        if isinstance(response, CallScript.Response): # replanning callback
            self.external_script_callback(response)
        elif response is not None and \
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
        if (self.uav_status.control_mode == 0) == value:
            self.get_logger().info(f'Setting UAV override to {value}')
            self.call_service(self.uav_override_service, self.uav_override_cli, Trigger.Request())

    def clear_uav_waypoints(self):
        # Clear existing UAV waypoints from all nodes
        self.get_logger().info(f'Clearing UAV waypoints')
        self.call_service(self.uav_waypoint_clear_service_1, self.uav_waypoint_clear_cli_1, Trigger.Request())
        self.call_service(self.uav_waypoint_clear_service_2, self.uav_waypoint_clear_cli_2, Trigger.Request())

    def send_uav_waypoint(self, position):
        # Send UAV waypoint service
        request = AddWaypoint.Request()
        request.wp.w = [position[0], -position[1], -position[2]] # world -> NED
        request.wp.hold_seconds = 1.0
        self.get_logger().info(f'Sending UAV waypoint [NED] {request.wp.w}')
        self.call_service(self.uav_waypoint_add_service, self.uav_waypoint_add_cli, request)

    def clear_ugv_waypoints(self):
        # Clear existing UGV waypoints
        self.get_logger().info(f'Clearing UGV waypoints')
        self.call_service(self.ugv_waypoint_clear_service, self.ugv_waypoint_clear_cli, Trigger.Request())

    def send_ugv_waypoint(self, position):
        # Construct a UGV waypoint message
        msg = Path()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        waypoint = PoseStamped()
        waypoint.pose.position.x = position[0]
        waypoint.pose.position.y = position[1]
        waypoint.pose.position.z = 0.0
        msg.poses = [waypoint]
        
        # Wait for subscriber
        self.get_logger().info(f'Waiting for subscriber on {self.ugv_waypoint_path_topic}...')
        while self.ugv_waypoint_path_pub.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.ugv_waypoint_path_pub.publish(msg)
        self.get_logger().info(f"Sent waypoint: {position} to {self.ugv_waypoint_path_topic}")

        return True

    def command_landing(self, value):
        request = SetBool.Request()
        request.data = value
        self.call_service(self.activate_landing_service, self.activate_landing_cli, request)

    def command_takeoff(self, value):
        # Tell takeoff commander to activate
        request = SetBool.Request()
        request.data = value
        self.call_service(self.activate_takeoff_service, self.activate_takeoff_cli, request)

    def activate_trajectory_override(self, position):
        # Send the override UAV goal position
        request = AddWaypoint.Request()
        request.wp.w = [position[0], -position[1], -position[2]] # world -> NED
        self.get_logger().info(f'Sending trajectory override [NED] {request.wp.w}')
        self.call_service(self.traj_override_service, self.traj_override_cli, request)

    def call_online_planner(self, together):
        # Construct input dict for online replanner
        if self.uav_takeoff_time is None:
            flightTime = 0
        else:
            flightTime = (self.get_clock().now() - self.uav_takeoff_time).to_msg().sec

        data = {
            'plannerParams': self.online_planner_params,
            'planParams': self.plan_params,
            'iTour': self.i_tour,
            'jTour': self.j_tour,
            'ugvIndex': 2 * self.i_tour + (0 if together else 1),
            'uavPos': self.uav_position,
            'ugvPos': [self.ugv_position.x, self.ugv_position.y],
            'flightTime': flightTime
        }

        # Call script service
        self.call_external_script('', data)

    def call_external_script(self, args, input_data=None, timeout=0.0):
        """
        Call external script with arguments
        
        Args:
            args: list of string arguments
            input_data: dict to send as JSON (optional)
            timeout: timeout in seconds (0 = use default)
        
        Returns:
            CallScript.Response or None if failed
        """
        request = CallScript.Request()
        request.args = args
        request.input_json = json.dumps(input_data) if input_data else ''
        request.timeout = timeout

        self.get_logger().info(f'Calling script with args: {args}')
        self.call_service(self.online_planner_service, self.online_planner_cli, request)

    def external_script_callback(self, response):
        # Call service (blocking)
        if response is not None:
            if response.success:
                self.get_logger().info(f'Script succeeded!')
                self.get_logger().info(f'Output: {response.stdout}')
            else:
                self.get_logger().error(f'Script failed: {response.stderr}')
                return
        else:
            self.get_logger().error('Service call failed')
            return None # continue with plan, i guess

        # Process response
        responseLines = [line.strip() for line in response.stdout.split("\n")]
        dictString = None
        for line in responseLines[::-1]:
            if len(line) > 0:
                dictString = line # take the last non-empty line
                break
        self.get_logger().info(f'Reading results dict: {dictString}')
        responseDict = json.loads(dictString)
        self.plan_params['uav_tours'] = responseDict['uavTours']
        self.plan_params['ugv_path'] = responseDict['ugvOrder']
        self.plan_params['ugv_point_map'] = responseDict['ugvPoints']


def main(args=None):
    rclpy.init(args=args)
    node = PlanManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
