# installs zenoh package for drone model development
#	this is necessary because the MRS repos use the zenoh routing protocol instead of default ros2 dds
#	and there are no precompiled binaries for zenoh on Ubuntu 24
read -p "### This script will install the zenoh package for the MRS drone gazebo simulation. [y/n] " input
if ! [[ $input =~ ^[yY]$ ]] ; then
	echo "Canceling"
	exit 0
fi

echo "### Installing Rust Toolchain ###"
sudo apt update
sudo apt install curl build-essential cmake pkg-config
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source ~/.cargo/env

echo "### Downloading Zenoh source ###"
git clone https://github.com/eclipse-zenoh/zenoh.git
cd zenoh

echo "### Building and installing tools ###"
cargo build --release --workspace
sudo install -m755 target/release/zenohd /usr/local/bin/
#sudo install -m755 target/release/zenoh-cli /usr/local/bin/ # does not exist for ubuntu 24

echo "### Testing outputs ###"
zenohd --version
#zenoh-cli --version
