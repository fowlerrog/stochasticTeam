
# python imports
import sys
import os.path
import json
import traceback
from itertools import product
from copy import deepcopy

# project imports
from OURS import *










def runPlannerFromParams(params, outputFolder):
	"""
	Run a planner from a parameter dict
	which will only produce ONE run
	"""

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
		cycles = create_cycles_CAHIT(tsp_points)

		# Solve UGV GTSP
		collect_to_cycle_times = compute_all_cycle_times(
			cycles,
			tsp_points
		)
		result = solve_gtsp_with_release_collect(
			points=tsp_points,
			cycles=cycles,
			start_point=params["START_POINT"],
			end_point=params["END_POINT"],
			collect_to_cycle_times=collect_to_cycle_times,
			plot=False,
			plotfilename=os.path.join(outputFolder, 'clusters.png')
		) # TODO pull plotting out of this with returned values

		# if params[showPlots or figureFolder is not None:
		# 	TSP_figure_name = filename=os.path.join(figureFolder, 'TSP_path.png')
		# 	if tsp_points:
		# 		plot_path(tsp_points,
		# 					filename=TSP_figure_name)
		# 	else:
		# 		# If no solution, just scatter the reordered points for reference
		# 		plot_points(points,
		# 					filename=TSP_figure_name)

		mission_time = result["total_cost"] / 100
		print("Total mission time (s):", mission_time)

		tsp_time = tsp_end_time - tsp_start_time
		glns_time = result["solver_time"] or -1  # fallback in case it's None
		cycle_tsp_total_time = -1  # or fill if you compute it elsewhere

		# file.write(f"{UGV_SPEED},{num_points},{seed},{mission_time:.2f},{tsp_time:.4f},{glns_time:.4f},{cycle_tsp_total_time:.4f}\n")
	except Exception:
		print("\nFailure during planning")
		print(traceback.format_exc())
		return

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

	# make sure independent variables are present
	independentVars = ["SEEDS","NUM_POINTS","UGV_SPEEDS"]
	notPresent = [k not in params for k in independentVars]
	if any(notPresent):
		print('Did not find required independent variables:\n\t',
		'\n\t'.join([independentVars[i] for i in range(len(notPresent)) if notPresent[i]]),
		'\nStopping run', sep='')
		return

	# generate all combinations of independent variables
	independentDict = {k:params[k] for k in independentVars}
	independentValueCombos = product(*[v for v in independentDict.values()])

	# call each combination

	for combo in independentValueCombos:
		# combine with other variables in params
		thisRunParams = deepcopy(params)
		thisRunIndParams = dict(zip(independentVars, combo))
		thisRunParams.update(thisRunIndParams)
		print('Running independent parameters:\n', thisRunIndParams, sep='')

		runPlannerFromParams(thisRunParams, absFolder)

		# collect dependent variables
		# write path to file
		# plot path

	# write overall results


### main function
if __name__ == '__main__':
	# print help message if necessary
	if len(sys.argv) < 2 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python runPlanner.py /path/to/run/settings.json [/path/to/run/settings2.json ...]')
		exit()
	
	# for each provided settings file, run planner
	for s in sys.argv[1:]:
		runPlannerFromSettings(s)
