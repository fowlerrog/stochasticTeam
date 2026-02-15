cd "$(ros2 pkg prefix roscopter)/share/roscopter/params"
ros2 service call /path_planner/load_mission_from_file rosflight_msgs/srv/ParamFile \
  "{filename: $(pwd)/multirotor_mission.yaml}" &&
ros2 service call /toggle_arm std_srvs/srv/Trigger &&
ros2 service call /toggle_override std_srvs/srv/Trigger
