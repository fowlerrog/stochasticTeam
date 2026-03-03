# installs ros2 packages for gazebo simulation
read -p "### This script will install ros2 packages for Gazebo simulation. [y/n] " input
if ! [[ $input =~ ^[yY]$ ]] ; then
	echo "Canceling"
	exit 0
fi

# save this script's path
SCRIPT=$(realpath "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(dirname "$SCRIPT")
echo "    This script is at $SCRIPT_DIR"

ROS_WS_FOLDER="ros_ws"

# check that wget and colcon are installed
echo "### Installing wget and colcon ###"
sudo apt-get update
sudo apt-get install wget colcon -y

# get gazebo
echo "### Installing gazebo ###"
sudo apt-get update
sudo apt-get install gz-harmonic ros-jazzy-ros-gz -y

# get turtlebot sim
echo "### Installing turtlebot4 ###"
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-jazzy-turtlebot4-simulator

# get drone sim
echo "### Cloning rosflight ###"
mkdir -p $SCRIPT_DIR/$ROS_WS_FOLDER/src
cd $SCRIPT_DIR/$ROS_WS_FOLDER/src
git clone https://github.com/rosflight/rosflight_ros_pkgs --recursive
git clone https://github.com/rosflight/roscopter

# install sim dependencies
echo "### Installing rosflight dependencies with rosdep ###"
source /opt/ros/jazzy/setup.bash
#source $SCRIPT_DIR/setup.bash
cd $SCRIPT_DIR/$ROS_WS_FOLDER
sudo rosdep init
rosdep update
rosdep install --from-path . -y --ignore-src

# build sim
echo "### Building rosflight sim with colcon ###"
cd $SCRIPT_DIR/$ROS_WS_FOLDER
# export PYTHON_EXECUTABLE=$SCRIPT_DIR/../.venv/bin/python3
# echo "PYTHON_EXECUTABLE: $PYTHON_EXECUTABLE"
# echo "CMAKE_PREFIX_PATH: $CMAKE_PREFIX_PATH"
colcon build
