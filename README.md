
This repo contains code to run various UAV-UGV planners.

# Dependencies:

Julia

various python packages as outlined in pythonEnv.txt

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

# Troubleshooting:

If you are having an environment issue, which may manifest itself as the pywrapcp optimizer not optimizing your TSP path solutions, create a python virtual environment (once):

(This requires pip and python3.9 to be installed, and is written for windows and bash.)

`chmod 777 setupPythonVirtualEnvironment.sh`

`bash setupPythonVirtualEnvironment.sh`

Source it (once per session)

`source .venv/Scripts/activate`

And then run your scripts normally:

`python scripts/runPlanner.py runs/test/settings.yaml`

To exit the venv:

`deactivate`

# Gurobi:

In order to use the Gurobi optimizer, you must have a Gurobi license. Free academic licenses are available at https://www.gurobi.com/academia/academic-program-and-licenses/

The gurobipy package installed as part of the python virtual environment does not include the licensing server that allows you to use the full academic license. It will only work on very small problems, unless you perform the full installation of gurobi on your system.

Once gurobi is installed and you have a license, go to your account page -> look at your licenses -> install this license. There should be a `grbgetkey` command to retrieve it to your system. Once this has been run, the program should be able to find the license and you can run any gurobi-related scripts.
