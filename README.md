
This repo contains code to run various UAV-UGV planners.

Dependencies:

Julia
various python packages

Structure:

/code contains python code
	runnable scripts begin with lowercase, whereas source code files begin with uppercase

/runs contains various runs
	*_settings.yaml files are lists of inputs. INDEPENDENT_VARIABLES define which variables will be varied across runs
	plan_time_results.yaml files are output timing information for benchmarking
	results_* folders are single-run path solutions

----

To use:

python runPlanner.py /path/to/settings.yaml
python plotPlan.py /path/to/results/folder/
python executePlan.py /path/to/execute_settings.yaml /path/to/plan_settings.yaml /path/to/plan/results/folder/

----

If you are having an environment issue, which may manifest itself as the pywrapcp optimizer not optimizing your TSP path solutions, create a python virtual environment (once):
(This requires pip and python3.9 to be installed, and is written for windows.)

chmod 777 setupPythonVirtualEnvironment.sh 
./setupPythonVirtualEnvironment.sh

Source it (once per session)

source .venv/Scripts/activate

And then run your scripts normally:

python code/runPlanner.py runs/test/settings.yaml

To exit the venv:

deactivate
