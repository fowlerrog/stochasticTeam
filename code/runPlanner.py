
# python imports
import sys
import time
import os
import json
import traceback
from itertools import product
from copy import deepcopy

# project imports
from ourPlanner import *

def runPlannerFromParams(params):
	"""
	Run a planner from a parameter dict (including run folder)
	which will only produce ONE run
	"""

	mission_time = None
	tsp_time = None
	glns_time = None
	cycle_tsp_total_time = None

	try:
		# Generate points
		points = generate_points(params["NUM_POINTS"],
								x_range=(0,params["SPACE_SIZE"]),
								y_range=(0,params["SPACE_SIZE"]),
								FIXED_Z=params["FIXED_Z"],
								seed=params["SEEDS"])

		# Solve TSP
		tsp_start_time = time.perf_counter()
		tsp_points = solve_tsp_with_fixed_start_end(
			points, params["START_POINT"], params["END_POINT"]
		)
		tsp_end_time = time.perf_counter()

		# Break into UAV cycles
		cycles = create_cycles_CAHIT(tsp_points, params)

		# Solve UGV GTSP
		collect_to_cycle_times = compute_all_cycle_times(
			cycles,
			tsp_points,
			params
		)
		result = solve_gtsp_with_release_collect(
			points=tsp_points,
			cycles=cycles,
			start_point=params["START_POINT"],
			end_point=params["END_POINT"],
			collect_to_cycle_times=collect_to_cycle_times,
			params=params
		)

		mission_time = result["total_cost"] / 100
		print("Total mission time (s):", mission_time)

		tsp_time = tsp_end_time - tsp_start_time
		glns_time = result["solver_time"] or -1  # fallback in case it's None
		cycle_tsp_total_time = -1  # or fill if you compute it elsewhere

		# Save if desired
		if "SAVE_PATH_FOLDER" in params:
			# construct save dict
			result.update({
				'tsp_points': tsp_points,
				'cycles': cycles
				})
			result['mapping_to_points'] = {k:list(v) for k,v in result["mapping_to_points"].items()} # json does not like np.array
			# create folder if it doesn't exist
			absSavePathFolder = os.path.join(params["RUN_FOLDER"], params["SAVE_PATH_FOLDER"])
			if not os.path.exists(absSavePathFolder):
				os.makedirs(absSavePathFolder)
			with open(os.path.join(absSavePathFolder, 'plan_results.json'), 'w') as f:
				json.dump(result, f)

	except Exception:
		print("\nFailure during planning")
		print(traceback.format_exc())

	return {
		"MISSION_TIME": mission_time,
		"TSP_TIME": tsp_time,
		"GLNS_TIME": glns_time,
		"CYCLE_TSP_TOTAL_TIME": cycle_tsp_total_time
	}

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

	defaultSettingsFileName = 'settings.json'
	if os.path.isdir(settingsFile):
		settingsFile = os.path.join(settingsFile, defaultSettingsFileName)
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
			thisRunParams["SAVE_PATH_FOLDER"] = 'results_' + '_'.join(['%s=%s'%(k,v) for k,v in thisRunIndParams.items()])

		# run
		thisRunOutput = runPlannerFromParams(thisRunParams)

		# store results
		independentResultsDict = appendDict(independentResultsDict, thisRunIndParams)
		dependentResultsDict = appendDict(dependentResultsDict, thisRunOutput)

	# write overall results
	independentResultsDict.update(dependentResultsDict) # combine
	with open(os.path.join(absFolder, 'our_cr2.json'), 'w') as f:
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
