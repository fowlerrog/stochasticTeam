import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # Get launch configuration values
    namespace = LaunchConfiguration('namespace').perform(context)
    fixed_frame = LaunchConfiguration('fixed_frame').perform(context)
    waypoints = LaunchConfiguration('waypoints').perform(context)

    uav_ugv_share_dir = get_package_share_directory('uav_ugv_teaming')
    param_file = os.path.join(uav_ugv_share_dir, 'params', 'uav_ugv_teaming_params.yaml')

    # Build frame names
    if namespace:
        odom_frame = f'{namespace}/odom'
        namespace_str = namespace
    else:
        odom_frame = 'odom'
        namespace_str = ''

    # Odometry publisher node
    odom_node = Node(
        package='uav_ugv_teaming',
        executable='simple_odom_publisher',
        name='simple_odom_publisher',
        namespace=namespace_str if namespace_str else None,
        output='screen'
    )
    
    # Waypoint navigator node
    navigator_node = Node(
        package='uav_ugv_teaming',
        executable='waypoint_navigator',
        name='waypoint_navigator',
        namespace=namespace_str if namespace_str else None,
        output='screen',
        parameters=[{'waypoints': waypoints}, param_file]
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
    
    # Robot state publisher - publishes TF for robot links and mesh visualization
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace_str if namespace_str else None,
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False,
            'frame_prefix': f'{namespace_str}/' if namespace_str else ''
        }]
    )

    # Static transform from world to odom (for multi-robot scenarios)
    world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom_publisher',
        namespace=LaunchConfiguration('namespace'),
        arguments=['0', '0', '0', '0', '0', '0', fixed_frame, odom_frame],
        output='screen'
    )

    return [
        world_to_odom,
        odom_node,
        navigator_node,
        robot_state_publisher,
    ]

def generate_launch_description():
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the robot'
    )

    fixed_frame_arg = DeclareLaunchArgument(
        'fixed_frame',
        default_value='world',
        description='Fixed/world frame name (e.g., world, map, odom)'
    )

    waypoints_arg = DeclareLaunchArgument(
        'waypoints',
        default_value='[1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 0.0]',
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
        namespace_arg,
        fixed_frame_arg,
        waypoints_arg,
        use_rviz_arg,
        OpaqueFunction(function=launch_setup),
        rviz_node,
    ])
