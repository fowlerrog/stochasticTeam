#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# import logging
# logging.root.setLevel(logging.DEBUG)

def generate_launch_description():

    rosflight_sim_share = get_package_share_directory("rosflight_sim")
    roscopter_sim_share = get_package_share_directory("roscopter_sim")
    uav_ugv_teaming_share = get_package_share_directory("uav_ugv_teaming")

    # Launch rosflight & roscopter upstream launch files

    rosflight_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rosflight_sim_share, "launch", "multirotor_standalone.launch.py")
        ),
        launch_arguments={
            'rviz2_config_file': os.path.join(uav_ugv_teaming_share, "rviz", "combined_config.rviz")
        }.items()
    )

    roscopter_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(roscopter_sim_share, "launch", "sim.launch.py")
        )
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

    # Perform param load AFTER rosflight_io starts
    # We call the service using the `ros2 service call` CLI
    load_params_cmd = ExecuteProcess(
        cmd=[
            "ros2", "service", "call",
            "/param_load_from_file",
            "rosflight_msgs/srv/ParamFile",
            "{filename: " + os.path.join(rosflight_sim_share, "params/multirotor_firmware/multirotor_combined.yaml") + "}"
        ],
        output="screen"
    )

    # Wait a couple seconds for rosflight_io to start
    load_params = TimerAction(
        period=6.0,       # seconds (increase if needed)
        actions=[load_params_cmd]
    )

    # also calibrate IMU
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

    return LaunchDescription([
        rosflight_sim_launch,
        roscopter_sim_launch,
        uav_ugv_teaming_launch,
        load_params,
        calibrate_imu
    ])
