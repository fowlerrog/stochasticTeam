# save this script's path
SCRIPT_DIR=$(dirname "${BASH_SOURCE[0]}")
echo "    This script is at $SCRIPT_DIR"

mkdir -p $SCRIPT_DIR/drone_ws/src
cd $SCRIPT_DIR/drone_ws/src
git clone -b ros2 https://github.com/NovoG93/sjtu_drone.git
cd ..
rosdep install -r -y --from-paths src --ignore-src --rosdistro=$ROS_DISTRO
colcon build --packages-select-regex sjtu*
