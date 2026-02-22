#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, OpaqueFunction, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

# import logging
# logging.root.setLevel(logging.DEBUG)

def generate_launch_description():

    rosflight_sim_share = get_package_share_directory("rosflight_sim")
    roscopter_sim_share = get_package_share_directory("roscopter_sim")
    uav_ugv_teaming_share = get_package_share_directory("uav_ugv_teaming")

    # Declare launch arguments
    param_file_arg = DeclareLaunchArgument(
        "param_file",
        default_value="default_run_params.yaml",
        description="Parameter set for execution"
    )
    param_file = LaunchConfiguration('param_file')

    plan_file_arg = DeclareLaunchArgument(
        "plan_file",
        default_value="",
        description="Path to execute_settings.yaml"
    )
    plan_file = LaunchConfiguration('plan_file')

    online_planner_file_arg = DeclareLaunchArgument(
        "online_planner_file",
        default_value="",
        description="Path to online_planner_settings.yaml"
    )
    online_planner_file = LaunchConfiguration('online_planner_file')

    # Launch rosflight & roscopter upstream launch files

    rosflight_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            # os.path.join(rosflight_sim_share, "launch", "multirotor_standalone.launch.py")
            'multirotor_standalone_modified.launch.py'
        ),
        launch_arguments={
            'rviz2_config_file': os.path.join(uav_ugv_teaming_share, "rviz", "combined_config_plan.rviz")
        }.items()
    )

    roscopter_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            # os.path.join(roscopter_sim_share, "launch", "sim.launch.py")
            'sim_modified.launch.py'
        ),
        launch_arguments={
            'state_topic': 'estimated_state_override'
        }.items()
    )

    # Launch simple waypoint follower UGV
    uav_ugv_teaming_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(uav_ugv_teaming_share, "launch", "simple_waypoint_nav.launch.py")
        ),
        launch_arguments={
            'waypoints':'[]',
            'use_rviz':'false',
            'namespace':'ugv',
            'fixed_frame':'world',
            'urdf_xacro':'../resources/jackal_description/urdf/jackal.urdf.xacro',
        }.items()
    )
    # with spinning wheels
    wheel_tf_publisher = Node(
        package='uav_ugv_teaming',
        executable='wheel_tf_publisher',
        name='wheel_tf_publisher',
        output='screen',
        parameters=[param_file]
    )

    # Launch plan manager
    plan_manager = Node(
        package='uav_ugv_teaming',
        executable='plan_manager',
        name='plan_manager',
        output='screen',
        parameters=[{
            'uav_odom_topic': 'estimated_state_override',
            'plan_filepath': plan_file,
            'online_planner_filepath': online_planner_file,
        }, param_file]
    )

    # Launch online replanner
    venv_script_caller = Node(
        package='uav_ugv_teaming',
        executable='venv_script_caller',
        name='venv_script_caller',
        output='screen',
        parameters=[{
            'venv_path': '../../.venv',
            'script_path': os.path.join(uav_ugv_teaming_share, "scripts", "standaloneOnlinePlanner.py"),
            'timeout': 60.0 # seconds
        }]
    )

    # Launch UAV mesh scaler
    mesh_scaler = Node(
        package='uav_ugv_teaming',
        executable='mesh_scaler',
        name='mesh_scaler',
        output='screen',
        parameters=[{
            'input_topic': '/rviz/mesh',
            'output_topic': '/rviz/mesh_scaled',
            'scale': 0.5,
        }]
    )

    # Launch collision force injector
    collision_force_injector = Node(
        package='uav_ugv_teaming',
        executable='collision_force_injector',
        name='collision_force_injector',
        output='screen',
        parameters=[param_file]
    )

    # Launch wrench collector
    wrench_collector = Node(
        package='uav_ugv_teaming',
        executable='wrench_collector',
        name='wrench_collector',
        output='screen',
    )

    # Launch state override
    state_override = Node(
        package='uav_ugv_teaming',
        executable='state_override',
        name='state_override',
        output='screen',
    )

    # Launch trajectory command override
    trajectory_command_override = Node(
        package='uav_ugv_teaming',
        executable='trajectory_command_override',
        name='trajectory_command_override',
        output='screen',
    )

    # Launch landing commander
    landing_commander = Node(
        package='uav_ugv_teaming',
        executable='landing_commander',
        name='landing_commander',
        output='screen',
        parameters=[{
            'uav_odom_topic': 'estimated_state_override',
            'raw_command_topic': 'raw_command_1',
            'command_topic': 'raw_command_2',
        }, param_file]
    )

    # Launch takeoff commander
    takeoff_commander = Node(
        package='uav_ugv_teaming',
        executable='takeoff_commander',
        name='takeoff_commander',
        output='screen',
        parameters=[{
            'uav_odom_topic': 'estimated_state_override',
            'raw_command_topic': 'raw_command_2',
            'command_topic': 'command',
        }, param_file]
    )

    # Random wind model
    wind_injector = Node(
        package='uav_ugv_teaming',
        executable='wind_injector',
        name='wind_injector',
        output='screen',
        parameters=[param_file],
    )

    # Perform param load AFTER rosflight_io starts
    # We call the service using the `ros2 service call` CLI
    load_rosflightio_params_cmd = ExecuteProcess(
        cmd=[
            "ros2", "service", "call",
            "/param_load_from_file",
            "rosflight_msgs/srv/ParamFile",
            "{filename: " + os.path.join(rosflight_sim_share, "params/multirotor_firmware/multirotor_combined.yaml") + "}"
        ],
        output="screen"
    )
    # Wait a couple seconds for rosflight_io to start
    load_rosflightio_params = TimerAction(
        period=3.0,       # seconds (increase if needed)
        actions=[load_rosflightio_params_cmd]
    )

    # Also calibrate IMU
    calibrate_imu_cmd = ExecuteProcess(
        cmd=["ros2", "service", "call",
             "/calibrate_imu",
             "std_srvs/srv/Trigger", "{}"],
        output="screen"
    )
    # give mixer time to load
    calibrate_imu = TimerAction(
        period=5.0,
        actions=[calibrate_imu_cmd]
    )

    # Move UAV to not collide with UGV
    uav_pos_cmd = ExecuteProcess(
        cmd=["ros2", "service", "call",
        "/dynamics/set_sim_state",
        "rosflight_msgs/srv/SetSimState",
        "{state: {pose: {position: {x: 1, y: 1, z: 0.01}, orientation: {x: 0, y: 0, z: 0.707, w: 0.707}}}}"],
        output="screen"
    )
    uav_pos = TimerAction(
        period=0.1,
        actions=[uav_pos_cmd]
    )

    return LaunchDescription([
        param_file_arg,
        plan_file_arg,
        online_planner_file_arg,
        rosflight_sim_launch,
        roscopter_sim_launch,
        uav_ugv_teaming_launch,
        wheel_tf_publisher,
        plan_manager,
        venv_script_caller,
        mesh_scaler,
        collision_force_injector,
        wrench_collector,
        state_override,
        trajectory_command_override,
        landing_commander,
        takeoff_commander,
        wind_injector,
        load_rosflightio_params,
        calibrate_imu,
        uav_pos,
    ])
