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

    execute_settings_file_arg = DeclareLaunchArgument(
        "execute_settings_file",
        default_value="",
        description="Path to execute_settings.yaml"
    )
    execute_settings_file = LaunchConfiguration('execute_settings_file')

    # Launch rosflight & roscopter upstream launch files

    rosflight_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rosflight_sim_share, "launch", "multirotor_standalone.launch.py")
        ),
        launch_arguments={
            'rviz2_config_file': os.path.join(uav_ugv_teaming_share, "rviz", "combined_config_plan.rviz")
        }.items()
    )

    roscopter_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(roscopter_sim_share, "launch", "sim.launch.py")
        ),
        launch_arguments={
            'state_topic': 'estimated_state_with_docking'
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
            'fixed_frame':'world'
        }.items()
    )

    # Launch docking manager
    docking_manager = Node(
        package='uav_ugv_teaming',
        executable='docking_manager',
        name='docking_manager',
        output='screen',
        parameters=[{
            'uav_odom_output_topic': 'estimated_state_with_docking',
            # 'uav_odom_topic': '/sim/roscopter/state'
        }, param_file]
    )

    # Launch plan manager
    plan_manager = Node(
        package='uav_ugv_teaming',
        executable='plan_manager',
        name='plan_manager',
        output='screen',
        parameters=[{
            'uav_odom_topic': 'estimated_state_with_docking',
            'plan_filepath': execute_settings_file
        }, param_file]
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

    # # Launch truth->gnss replacer
    # gnss_replacer = Node(
    #     package='uav_ugv_teaming',
    #     executable='gnss_replacer',
    #     name='gnss_replacer',
    #     output='screen',
    #     # parameters=[{
    #     #     'gnss_topic': '/gnss_truth'
    #     # }]
    # )

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

    # # Increase trajectory follower integral coefficients
    # traj_follower_cmd = ExecuteProcess(
    #     cmd=["ros2", "service", "call",
    #     "/trajectory_follower/set_parameters",
    #     "rcl_interfaces/srv/SetParameters",
    #     "{parameters: [{name: 'u_d_ki', value: {type: 3, double_value: 0.1}}, {name: 'u_n_ki', value: {type: 3, double_value: 0.02}}, {name: 'u_e_ki', value: {type: 3, double_value: 0.02}}]}"],
    #     output="screen"
    # )
    # traj_follower = TimerAction(
    #     period=1.0,
    #     actions=[traj_follower_cmd]
    # )

    # Move UAV to not collide with UGV
    uav_pos_cmd = ExecuteProcess(
        cmd=["ros2", "service", "call",
        "/dynamics/set_sim_state",
        "rosflight_msgs/srv/SetSimState",
        "{state: {pose: {position: {x: 1, y: 1, z: 0}}}}"],
        output="screen"
    )
    uav_pos = TimerAction(
        period=0.1,
        actions=[uav_pos_cmd]
    )

    return LaunchDescription([
        param_file_arg,
        execute_settings_file_arg,
        rosflight_sim_launch,
        roscopter_sim_launch,
        uav_ugv_teaming_launch,
        docking_manager,
        plan_manager,
        mesh_scaler,
        # gnss_replacer,
        load_rosflightio_params,
        calibrate_imu,
        # traj_follower,
        uav_pos,
    ])
