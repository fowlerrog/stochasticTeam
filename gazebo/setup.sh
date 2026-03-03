
# puts us in the python virtual environment, and sources ros

SCRIPT=$(realpath "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(dirname "$SCRIPT")
echo "    This script is at $SCRIPT_DIR"

#echo "Entering python venv @ $SCRIPT_DIR/../.venv/bin/activate"
#source $SCRIPT_DIR/../.venv/bin/activate

echo "Sourcing ROS2 @ /opt/ros/jazzy/setup.bash"
source /opt/ros/jazzy/setup.bash

echo "Sourcing ros_ws @ $SCRIPT_DIR/ros_ws/install/setup.bash"
source $SCRIPT_DIR/ros_ws/install/setup.bash
