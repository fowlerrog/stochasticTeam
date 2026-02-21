
This repo contains code to run various UAV-UGV planners. Setting up the Python Virtual Environment is recommended.

# Dependencies:

Julia (for deterministic planner)

various python packages as listed in pythonEnv.txt

For gazebo simulator:

ROS2 : https://docs.ros.org/en/jazzy/index.html

Turtlebot 4 : https://github.com/turtlebot/turtlebot4_simulator

Rosflight / Roscopter : https://github.com/rosflight/roscopter

# Structure:

/src/pathPlanning contains a python module for source code

/scripts contains runnable python scripts

- get run instructions by running: $ python scripts/scriptName.py -h

/runs contains various runs

- *_settings.yaml files are lists of inputs.
	- INDEPENDENT_VARIABLES define which variables will be varied across runs
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

# ROS2 Installation:

When installing the Gazebo simulator/ROS2, colcon does not seem to find packages or Python3 properly, so do not have the Python virtual environment active when building or running. Calls to the online planner are done by the Node independently entering and exiting the venv.

The simulation relies on external model repositories, and on Gazebo and ROS2 itself. Install these packages for the first time:

`cd gazebo`

`./installRos2Jazzy.sh`

`./installGazeboHarmonic.sh`

Then install and build the gazebo models, including our package:

`./installGazeboRepos.sh`

This automatically calls `colcon build` in `gazebo/ros_ws`, but anytime you change the uav_ugv_teaming ros package, you will have to rebuild by doing this again.

You will also need the Jackal URDF and STL models:

`./installJackalModels.sh`

The script setup.sh is provided for your convenience which sources ROS2 and the local packages.

`source setup.sh`

# Running Gazebo

Launch files are in `gazebo\launch`

To launch the full simulation with no plan:

`ros2 launch full_sim.launch.py`

To launch the simulation with an offline plan, and optionally with an online planner:

`ros2 launch run_with_plan.launch.py`

After it has stopped printing new messages (except for occasional status updates) you can then activate the plan manager. The first time it will just prepare for execution by landing the UAV on the UGV. Once this is done and it prints READY, another activation will actually begin plan execution. The command for this is:

`ros2 service call /plan_manager/start std_srvs/srv/Trigger`

# Gurobi:

In order to use the Gurobi optimizer, you must have a Gurobi license. Free academic licenses are available at https://www.gurobi.com/academia/academic-program-and-licenses/

The gurobipy package installed as part of the python virtual environment does not include the licensing server that allows you to use the full academic license. It will only work on very small problems, unless you perform the full installation of gurobi on your system.

Once gurobi is installed and you have a license, go to your account page -> look at your licenses -> install this license. There should be a `grbgetkey` command to retrieve it to your system. Once this has been run, the program should be able to find the license and you can run any gurobi-related scripts.
