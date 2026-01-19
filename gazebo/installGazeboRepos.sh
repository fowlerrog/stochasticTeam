# installs ros2 packages for gazebo simulation
read -p "### This script will install ros2 packages for gazebo simulation. [y/n] " input
if ! [[ $input =~ ^[yY]$ ]] ; then
	echo "Canceling"
	exit 0
fi

# get gazebo sim
echo "### Installing wget, gazebo, and turtlebot4"
sudo apt-get update && sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gz-harmonic ros-jazzy-turtlebot4-simulator
