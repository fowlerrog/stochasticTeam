import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    single_robot_launch = PathJoinSubstitution([
        FindPackageShare('uav_ugv_teaming'),
        'launch',
        'simple_waypoint_nav.launch.py'
    ])
    
    robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(single_robot_launch),
        launch_arguments={
            'namespace': 'robot1',
            'x': '0.0',  # Starting x position
            'y': '0.0',  # Starting y position
            'yaw': '0.0',  # Starting orientation
            'fixed_frame': 'world',  # Match your external package
            'waypoints': '[1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 0.0]',
            'use_rviz': 'false'
        }.items()
    )
    
    robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(single_robot_launch),
        launch_arguments={
            'namespace': 'robot2',
            'x': '3.0',  # Different starting position
            'y': '0.0',
            'yaw': '0.0',
            'fixed_frame': 'world',  # Match your external package
            'waypoints': '[2.0, 0.0, 2.0, 2.0, 0.0, 2.0, 0.0, 0.0]',
            'use_rviz': 'false'
        }.items()
    )
    
    return LaunchDescription([robot1, robot2])
