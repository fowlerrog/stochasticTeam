
This repo contains code to run various UAV-UGV planners.

# Dependencies:

Julia (for deterministic planner)

various python packages as listed in pythonEnv.txt

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

(This requires pip and python<=3.9 to be installed, and is written for bash. Using python>3.9 may cause the ortools TSP solver to not function.)

`chmod 777 setupPythonVirtualEnvironment.sh`

`bash setupPythonVirtualEnvironment.sh`

Source it (once per session)

`source .venv/Scripts/activate` on windows
`source .venv/bin/activate` on linux

And then run your scripts normally:

`python scripts/runPlanner.py runs/test/settings.yaml`

To exit the venv:

`deactivate`
