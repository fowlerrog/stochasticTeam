# bash script to create python virtual environment with specific versions of packages

# detect OS type for pip and python locations
if [[ "$OSTYPE" == "msys" ]]; then
	# linux emulator on Windows e.g. git bash
	pipLocation=".venv/Scripts/pip.exe"
	allpython3s=($(which -a python3))
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
	# ubuntu
	pipLocation=".venv/bin/pip"
	allpython3s=($(find /bin/ -type f -executable -name "python3*"))
else
	echo "OS type $OSTYPE not recognized, exiting"
	exit 0
fi

# ask user which python version to use
echo "Found ${#allpython3s[@]} Python 3.x versions. Python <= 3.9 recommended."
i=0
for pyver in "${allpython3s[@]}"; do
	echo "$i : $pyver : $($pyver --version)";
	let "i++";
done

read -p "Please choose one: [0-N] " index
if ! [[ $index =~ ^[0-9]+$ ]] || [ "$index" -lt 0 ] || [ "$index" -ge "$i" ] ; then
	echo "Input not valid, exiting"
	exit 0
fi

pythonLocation="${allpython3s[$index]}"
echo "User chose interpreter at $pythonLocation"

# install virtualenv
echo "Installing virtualenv"
if command -v virtualenv ; then
	echo "virtualenv already installed"
else
	# need to install virtualenv
	sudo pip install virtualenv
	if command -v virtualenv ; then
		echo "Installed with pip"
	else
		# pip might fail, depending on the user package management; also try apt
		sudo apt install python3-virtualenv
		if command -v virtualenv ; then
			echo "Installed with apt"
		else
			echo "Failed to install virtualenv, exiting"
			exit 0
		fi
	fi
fi

echo "Trying to create virtual environment"
virtualenv -p $pythonLocation .venv && \
$pipLocation install -r pythonEnv.txt && \
$pipLocation install -e . && \
echo '########    Successfully created Python venv!    ########'
