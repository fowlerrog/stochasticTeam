# installs Gazebo Harmonic on ubuntu
read -p "### This script will install Gazebo Harmonic. [y/n] " input
if ! [[ $input =~ ^[yY]$ ]] ; then
	echo "Canceling"
	exit 0
fi

echo "### Installing curl, lsb-release, gnupg ###"
sudo apt-get update
sudo apt-get install curl lsb-release gnupg -y

echo "### Installing Gazebo ###"
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic ros-jazzy-ros-gz -y
