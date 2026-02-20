import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    diagnostic_logger = Node(
        package='uav_ugv_teaming',
        executable='diagnostic_logger',
        name='diagnostic_logger',
        output='screen',
    )

    return LaunchDescription([diagnostic_logger])
