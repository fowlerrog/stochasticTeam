import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    waypoints_arg = DeclareLaunchArgument(
        'waypoints',
        default_value="'[1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 0.0]'",
        description='List of waypoints as [x1, y1, x2, y2, ...]'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    # Path to RViz config
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('uav_ugv_teaming'),
        'rviz',
        'simple_waypoint_nav_config.rviz'
    ])

    # Odometry publisher node
    odom_node = Node(
        package='uav_ugv_teaming',
        executable='simple_odom_publisher',
        name='simple_odom_publisher',
        output='screen'
    )
    
    # Waypoint navigator node
    navigator_node = Node(
        package='uav_ugv_teaming',
        executable='waypoint_navigator',
        name='waypoint_navigator',
        output='screen',
        parameters=[{
            'waypoints': LaunchConfiguration('waypoints'),
            'linear_speed': 0.2,
            'angular_speed': 0.5,
            'distance_tolerance': 0.1
        }]
    )

    # Robot state publisher (publishes TF for robot_description)
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('turtlebot4_description'),
            'urdf',
            'standard',
            'turtlebot4.urdf.xacro'
        ])
    ])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False
        }]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        waypoints_arg,
        use_rviz_arg,
        odom_node,
        navigator_node,
        robot_state_publisher,
        rviz_node,
    ])
