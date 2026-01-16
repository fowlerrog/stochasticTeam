# bash script to create python virtual environment with specific versions of packages

# find python 3.9 installation
allpython3s=($(which -a python3))
echo "Found ${#allpython3s[@]} Python 3.x versions. Python <= 3.9 recommended."
i=0
for pyver in "${allpython3s[@]}"; do echo "$i : $pyver : $($pyver --version)"; let "i++"; done

read -p "Please choose one: [0-N] " index
if ! [[ $index =~ ^[0-9]+$ ]] || [ "$index" -lt 0 ] || [ "$index" -ge "$i" ] ; then
	echo "Input not valid, exiting"
	exit 0
fi

pythonLocation="${allpython3s[$index]}"
echo "User chose interpreter at $pythonLocation"

# detect OS type for pip location
if [[ "$OSTYPE" == "msys" ]]; then
	# linux emulator on Windows e.g. git bash
	pipLocation=".venv/Scripts/pip.exe"
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
	# ubuntu
	pipLocation=".venv/bin/pip"
else
	echo "OS type $OSTYPE not recognized, exiting"
	exit 0
fi

# install virtualenv and create virtual environment
sudo pip install virtualenv && \
virtualenv -p $pythonLocation .venv && \
$pipLocation install -r pythonEnv.txt && \
$pipLocation install -e . && \
echo '########    Successfully created Python venv!    ########'
