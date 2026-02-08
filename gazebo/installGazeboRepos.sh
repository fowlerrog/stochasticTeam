# installs ros2 packages for gazebo simulation
read -p "### This script will install ros2 packages for Gazebo simulation. [y/n] " input
if ! [[ $input =~ ^[yY]$ ]] ; then
	echo "Canceling"
	exit 0
fi

# save this script's path
SCRIPT_DIR=$(dirname "${BASH_SOURCE[0]}")
echo "    This script is at $SCRIPT_DIR"

# check that wget and colcon are installed
echo "### Installing wget and colcon ###"
sudo apt-get update
sudo apt-get install wget colcon -y

# get gazebo
echo "### Installing gazebo ###"
sudo apt-get update
sudo apt-get install gz-harmonic ros-jazzy-gazebo-ros-pkgs -y

# get jackal sim
echo "### Installing Jackal sim ###"
wget https://packages.clearpathrobotics.com/public.key -O - | sudo apt-key add -
sudo sh -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list'
sudo apt-get update
sudo apt-get install ros-foxy-jackal-desktop ros-foxy-jackal-simulator -y

# Source - https://stackoverflow.com/a/22592801
if [ $(dpkg-query -W -f='${Status}' ros-foxy-jackal-desktop 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
	echo "### Failed to install prebuilt deb package (may not be available for your OS version) ###"
	echo "### Installing from source ###"
	mkdir -p $SCRIPT_DIR/jackal_ws/src
	cd $SCRIPT_DIR/jackal_ws/src
	git clone -b foxy-devel https://github.com/jackal/jackal.git
	git clone -b foxy-devel https://github.com/jackal/jackal_desktop.git
	git clone -b foxy-devel https://github.com/jackal/jackal_simulator.git
	cd ..
	source /opt/ros/jazzy/setup.bash
	rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO
	colcon build
	source install/setup.bash
fi


# get drone sim
echo "### Cloning and building SJTU Drone simulator ###"
