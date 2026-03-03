
# Add a single waypoint at (3.0, 2.0)
ros2 topic pub --once /ugv/waypoint geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'world'},
  pose: {
    position: {x: 3.0, y: 2.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"

# Replace entire path with new waypoints
ros2 topic pub --once /ugv/waypoint_path nav_msgs/msg/Path "{
  header: {frame_id: 'world'},
  poses: [
    {pose: {position: {x: 1.0, y: 0.0, z: 0.0}}},
    {pose: {position: {x: 2.0, y: 1.0, z: 0.0}}},
    {pose: {position: {x: 3.0, y: 2.0, z: 0.0}}}
  ]
}"
