# installs ros2 packages for gazebo simulation
read -p "### This script will install ros2 packages for gazebo simulation. [y/n] " input
if ! [[ $input =~ ^[yY]$ ]] ; then
	echo "Canceling"
	exit 0
fi

# get gazebo sim
#echo "### Installing wget, gazebo, and turtlebot4"
#sudo apt-get update && sudo apt-get install wget
#sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
#wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
#sudo apt-get update
#sudo apt-get install gz-harmonic ros-jazzy-turtlebot4-simulator

# get PX4 sim
# PX4 also requires python packages empy, pyros-genmsg, setuptools to run; which this should install
#echo "### Cloning and making PX4 simulator"
#git clone https://github.com/PX4/PX4-Autopilot.git --recursive
#bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools
#cd PX4-Autopilot
#make px4_sitl

# get UAV models
echo "### Installing MRS UAV System repo"
curl https://ctu-mrs.github.io/ppa2-stable/add_ppa.sh | bash
sudo apt install ros-jazzy-mrs-uav-system-full
