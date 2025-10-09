# bash script to create python virtual environment with specific versions of packages

pip install virtualenv && \
virtualenv -p python3.9 .venv && \
.venv/Scripts/pip.exe install -r pythonEnv.txt && \
echo '########    Successfully created Python venv!    ########'
