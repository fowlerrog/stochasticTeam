# PROSPECT

**Probabilistically Safe Scalable Planning for Energy-Aware Coordinated UAV-UGV Teams in Stochastic Environments**

[![Paper](https://img.shields.io/badge/paper-IROS%202026-1f6feb)](https://doi.org/10.48550/arXiv.2604.02142)
[![arXiv](https://img.shields.io/badge/arXiv-2603.22507-b31b1b)](https://doi.org/10.48550/arXiv.2604.02142)
[![Video](https://img.shields.io/badge/video-experiments-c4302b)](https://youtu.be/w81nHt7OAxY)
[![Thesis](https://img.shields.io/badge/thesis-Northeastern_University)](https://www.proquest.com/openview/aa1f80277bbcf0c2e0944fc5b1452d8d)

Code for our paper in the *IEEE/RSJ International Conference on Intelligent Robots and Systems 2026 (IROS 2026)*: PRO-SPECT,
including python scripts for planning, numerical simulation, and printing of stastics; and a ROS2 simulation.
Developed at the [Multi-Agent Robotics and Autonomy Lab](https://github.com/Multi-Agent-Robotics-and-Autonomy-Lab), Northeastern University.
Work done in association with The Charles Stark Draper Laboratory, Cambridge MA.

## Abstract

We consider energy-aware planning for an unmanned aerial vehicle (UAV) and unmanned ground vehicle (UGV) team operating in a stochastic environment. The UAV must visit a set of air points in minimum time while respecting energy constraints, relying on the UGV as a mobile charging station. Unlike prior work that assumed deterministic travel times or used fixed robustness margins, we model travel times as random variables and bound the probability of failure (energy depletion) across the entire mission to a user-specified risk level. We formulate the problem as a Mixed-Integer Program and propose PRO-SPECT, a polynomial-time algorithm that generates risk-bounded plans. The algorithm supports both offline planning and online re-planning, enabling the team to adapt to disturbances while preserving the risk bound. We provide theoretical results on solution feasibility and time complexity. We also demonstrate the performance of our method via numerical comparisons and simulations.

## Algorithm Overview

Given a set of points to be visited in the air, a ground space traversable by a UGV, and a model to estimate the travel time mean and variance between any two points:

1. Solve a Traveling Salesperson Problem to estimate a minimum-time path between all air points, using the mean air travel time estimator.
2. Split the path into multiple tours with a Dynamic Program that minimizes failure risk. Return a solution once the risk drops below a user-specified threshold.
3. (Optionally) refine each tour by solving a TSP on only its points.

Since this algorithm runs in polynomial time, it can also be evaluated on the fly for online replanning. A receding horizon is used to reduce solve times.

## Repository Structure

/src/pathPlanning contains a python module for source code

/scripts contains runnable python scripts
- get run instructions by running: $ python scripts/scriptName.py -h

/runs contains the results of various runs
- *_settings.yaml files are lists of inputs.
	- INDEPENDENT_VARIABLES define which variables will be varied across runs
- plan_time_results.yaml files are output timing information for benchmarking
- results_* folders are single-run path solutions

/testCases contain pytest cases for code verification

## Dependencies

This repo contains code to run various UAV-UGV planners.
Setting up the Python Virtual Environment is recommended. Bash scripts are provided for this purpose.
Python scripts should be platform-agnostic. ROS2 will only run on Linux.

- Various python packages as listed in pythonEnv.txt
- Julia (for deterministic planner)
- For gazebo simulator:
	- ROS2 : https://docs.ros.org/en/jazzy/index.html
	- Turtlebot 4 : https://github.com/turtlebot/turtlebot4_simulator
	- Rosflight / Roscopter : https://github.com/rosflight/roscopter

## Usage

Running the planner requires a set of .yaml files that specify planner settings. These files can be combined, but separating them allows for reuse without redundancy.
- env_settings.yaml contains the environmental models used to plan or execute
- plan_settings.yaml contains planner parameters
- execute_settings.yaml contains plan execution settings

Scripts generally contain their own instructions and a short description. For any of them, you can get a short set of arguments with:

`python scripts/script.py -h`

Running the offline planner is as simple as providing a path to the plan settings file.

`python scripts/planPath.py /path/to/plan_settings.yaml`

Plotting the results of planning is also simple.

`python scripts/plotPlan.py /path/to/results/folder/`

Executing a plan requires pointing to execution settings, the original plan settings, and the results folder containing the plan you're executing on.

`python scripts/executePlan.py /path/to/execute_settings.yaml /path/to/plan_settings.yaml /path/to/plan/results/folder/`

Many scripts will accept many repeated arguments, such as executing many plans with the same execution settings.

# Comparison Cases

Scripts for running comparison cases are contained in scripts/annealing/, scripts/branch_and_cut/, and scripts/terra/. These are implemented individually.
Their accompanying process_* scripts should process their output files into usable numbers for comparison; these should be the numbers in the paper tables.

scripts/paper_run_cases.txt details which input files recreate which tables. They are labelled according to the thesis version, not the IROS paper, but the contents are generally the same.

## Setting up Python Virtual Environment

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

`python scripts/planPath.py runs/test/plan_settings.yaml`

To exit the venv:

`deactivate`

## ROS2 Installation

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

## Running Gazebo

Launch files are in `gazebo\launch`

To launch the full simulation with no plan:

`ros2 launch full_sim.launch.py`

To launch the simulation with an offline plan, and optionally with an online planner:

`ros2 launch run_with_plan.launch.py`

After it has stopped printing new messages (except for occasional status updates) you can then activate the plan manager. The first time it will just prepare for execution by landing the UAV on the UGV. Once this is done and it prints READY, another activation will actually begin plan execution. The command for this is:

`ros2 service call /plan_manager/start std_srvs/srv/Trigger`

## Gurobi

In order to use the Gurobi optimizer, you must have a Gurobi license. Free academic licenses are available at https://www.gurobi.com/academia/academic-program-and-licenses/

The gurobipy package installed as part of the python virtual environment does not include the licensing server that allows you to use the full academic license. It will only work on very small problems, unless you perform the full installation of gurobi on your system.

Once gurobi is installed and you have a license, go to your account page -> look at your licenses -> install this license. There should be a `grbgetkey` command to retrieve it to your system. Once this has been run, the program should be able to find the license and you can run any gurobi-related scripts.
