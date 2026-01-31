
# puts us in the python virtual environment, and sources ros

SCRIPT_DIR=$(dirname "${BASH_SOURCE[0]}")
echo "This script is at $SCRIPT_DIR"

echo "Entering python venv"
source $SCRIPT_DIR/../.venv/bin/activate

echo "Sourcing ROS2"
source /opt/ros/jazzy/setup.bash
