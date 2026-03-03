#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    rosflight_sim_share = get_package_share_directory("rosflight_sim")
    roscopter_sim_share = get_package_share_directory("roscopter_sim")

    #
    # 1. Launch stock upstream launch files
    #

    rosflight_sim_launch = IncludeLaunchDescription(
        # PythonLaunchDescriptionSource(
        #     os.path.join(os.path.dirname(__file__), "wrapper_standalone.launch.py")
        # )
        PythonLaunchDescriptionSource(
            os.path.join(rosflight_sim_share, "launch", "multirotor_standalone.launch.py")
        )
    )

    roscopter_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(roscopter_sim_share, "launch", "sim.launch.py")
        )
    )

    #
    # 2. Perform param load AFTER rosflight_io starts
    #

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
        load_params,
        calibrate_imu
    ])
