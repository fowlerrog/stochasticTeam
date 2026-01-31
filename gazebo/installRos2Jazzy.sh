# installs ROS2 Jazzy on ubuntu
read -p "### This script will install ros2 jazzy. [y/n] " input
if ! [[ $input =~ ^[yY]$ ]] ; then
	echo "Canceling"
	exit 0
fi

echo "### Enabling Ubuntu Universe repository"
sudo apt install software-properties-common
sudo add-apt-repository universe

echo "### Downloading ROS2 debian package"
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

echo "### Installing ROS2"
sudo apt update && sudo apt upgrade
sudo apt install ros-jazzy-desktop
