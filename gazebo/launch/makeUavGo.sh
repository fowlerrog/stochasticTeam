ros2 service call /toggle_arm std_srvs/srv/Trigger &&
ros2 service call /toggle_override std_srvs/srv/Trigger &&
ros2 param set /path_planner num_waypoints_to_publish_at_start 100
