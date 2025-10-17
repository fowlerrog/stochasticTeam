
# python imports
import sys
import os
import json
import traceback
from itertools import product
from copy import deepcopy

# project imports
from NodeUtils import *
from OurPlanner import OurPlanner
from Constants import planSettingsFilename, planTimeResultsFilename

def runPlannerFromParams(params):
	"""
	Run a planner from a parameter dict (including run folder)
	which will only produce ONE run
	"""

	# Generate points
	points = generate_points(params["NUM_POINTS"],
							x_range=(0,params["SPACE_SIZE"]),
							y_range=(0,params["SPACE_SIZE"]),
							FIXED_Z=params["FIXED_Z"],
							seed=params["SEEDS"])

	# Construct and call solver
	ourPlanner = OurPlanner(params)
	ourPlanner.solve(points)

	# Save if desired
	if "SAVE_PATH_FOLDER" in params:
		ourPlanner.print_results_to_json()

	return ourPlanner.time_info

def appendDict(d1, d2):
	"""Appends the values in d2 to the values of d1, for matching keys"""
	for k in d2.keys():
		if k in d1:
			d1[k].append(d2[k])
		else:
			d1[k] = [d2[k]]
	return d1

def runPlannerFromSettings(settingsFile):
	"""
	Run a planner from a settings file (or folder) path
	which may generate several sets of parameters for separate runs
	"""

	if os.path.isdir(settingsFile):
		settingsFile = os.path.join(settingsFile, planSettingsFilename)
	print('Planning from settings', settingsFile)
	absFile = os.path.abspath(settingsFile)
	absFolder = os.path.dirname(absFile)
    
	# load run parameters from json
	params = {}
	with open(absFile, 'r') as f:
		try:
			params = json.load(f)
		except Exception:
			print(traceback.format_exc())
	if len(params) == 0:
		print('Params not found')
		return

	params["RUN_FOLDER"] = absFolder

	# make sure independent variables are present
	independentVars = params["INDEPENDENT_VARIABLES"]
	notPresent = [k not in params for k in independentVars]
	if any(notPresent):
		print('Did not find required independent variables:\n\t',
		'\n\t'.join([independentVars[i] for i in range(len(notPresent)) if notPresent[i]]),
		'\nStopping run', sep='')
		return

	# generate all combinations of independent variables
	independentDict = {k:params[k] for k in independentVars}
	independentValueCombos = product(*[v for v in independentDict.values()])

	# set up run result lists
	independentResultsDict = {}
	dependentResultsDict = {}

	# call each combination
	for combo in independentValueCombos:
		# combine with other variables in params
		thisRunParams = deepcopy(params)
		thisRunIndParams = dict(zip(independentVars, combo))
		thisRunParams.update(thisRunIndParams)
		print('Running independent parameters:\n', thisRunIndParams, sep='')

		# set results save location, if desired
		if "SAVE_PATHS" in params and params["SAVE_PATHS"]:
			thisRunParams["SAVE_PATH_FOLDER"] = 'results_' + '_'.join(['%s_%s'%(k,v) for k,v in thisRunIndParams.items()])

		# run
		thisRunOutput = runPlannerFromParams(thisRunParams)

		# store results
		independentResultsDict = appendDict(independentResultsDict, thisRunIndParams)
		dependentResultsDict = appendDict(dependentResultsDict, thisRunOutput)

	# write overall results
	independentResultsDict.update(dependentResultsDict) # combine
	with open(os.path.join(absFolder, planTimeResultsFilename), 'w') as f:
		json.dump(independentResultsDict, f)

### main function
if __name__ == '__main__':
	# print help message if necessary
	if len(sys.argv) < 2 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python runPlanner.py /path/to/run/settings.json [/path/to/run/settings2.json ...]')
		exit()
	
	# for each provided settings file, run planner
	for s in sys.argv[1:]:
		try:
			runPlannerFromSettings(s)
		except Exception:
			print(traceback.format_exc())
