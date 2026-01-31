
This repo contains code to run various UAV-UGV planners. Setting up the Python Virtual Environment is recommended.

# Dependencies:

Julia (for deterministic planner)

various python packages as listed in pythonEnv.txt

For gazebo simulator:

ROS2 : https://docs.ros.org/en/jazzy/index.html

Turtlebot 4 : https://github.com/turtlebot/turtlebot4_simulator

MRS UAV System : https://github.com/ctu-mrs/mrs_uav_system/tree/ros2

# Structure:

/src/pathPlanning contains a python module for source code

/scripts contains runnable python scripts

- get run instructions by running: $ python scripts/scriptName.py -h

/runs contains various runs

- *_settings.yaml files are lists of inputs.
	- INDEPENDENT_VARIABLES define which variables will be varied across runs
	- settings grouped together in this list will be correlated when generating runs 
- plan_time_results.yaml files are output timing information for benchmarking
- results_* folders are single-run path solutions

/testCases contain pytest cases

# Usage:

python scripts/runPlanner.py /path/to/settings.yaml

python scripts/plotPlan.py /path/to/results/folder/

python scripts/executePlan.py /path/to/execute_settings.yaml /path/to/plan_settings.yaml /path/to/plan/results/folder/

# Setting up Python Virtual Environment:

If you are having an environment issue, which may manifest itself as the pywrapcp optimizer not optimizing your TSP path solutions, create a python virtual environment (once):

(This requires pip and python<=3.9 to be installed, and is written for bash. Using python>3.9 may cause the ortools TSP solver to not function.)

Installing a previous version of python can be done with the deadsnakes repository:

`sudo add-apt-repository 'ppa:deadsnakes/ppa' && sudo apt-get update && sudo apt-get install python3.9`

Once you have an appropriate Python version installed:

`chmod 777 setupPythonVirtualEnvironment.sh`

`bash setupPythonVirtualEnvironment.sh`

If the above gives you "ModuleNotFoundError: No module named 'distutils.cmd'", your default python is above Python 3.12 where distutils is deprecated in favor of setuptools. Fix this by installing distutils for your chosen Python .venv version:

`python3.9 -m pip install distutils`

Source it (once per session)

`source .venv/Scripts/activate` on windows

`source .venv/bin/activate` on linux

And then run your scripts normally:

`python scripts/runPlanner.py runs/test/settings.yaml`

To exit the venv:

`deactivate`

# Gazebo/ROS2 Installation:

Make sure you are inside the Python VM when using the Gazebo simulator, and also when installing the Gazebo simulator/ROS2. This is to avoid any missing python packages inside the virtual environment.

The Gazebo simulation relies on external Gazebo model repositories, and on Gazebo and ROS2 itself. Install these packages for the first time:

`cd gazebo`

`./installRos2Jazzy.sh`

Before installing the gazebo models, set up your environment by sourcing the setup file:

(You also have to source this file at the start of each session to use ROS2)

`source /opt/ros/jazzy/setup.bash`

The install the gazebo models:

`./installGazeboRepos.sh`

And source again to get the new packages:

`source /opt/ros/jazzy/setup.bash`

The convenience script setup.sh is provided for your convenience, which puts you in the Python VM and sources ROS2.

`source setup.sh`
