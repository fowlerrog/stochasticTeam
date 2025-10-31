# python imports
import sys
import os
from itertools import product
from copy import deepcopy
from random import seed

# project imports
from .NodeUtils import *
from .Constants import planSettingsFilename, planTimeResultsFilename
from .RunnerUtils import loadYamlContents, appendDict, toDir, writeYaml
from .OurPlanner import *

def plannerFromParams(params):
	plannerClass = getattr(sys.modules[__name__], params['PLANNER_TYPE'])
	return plannerClass(params)

def runPlannerFromParams(params):
	"""
	Run a planner from a parameter dict (including run folder)
	which will only produce ONE run
	"""

	# Set seed if specified
	if 'SEED' in params and params['SEED'] is not None:
		seed(params['SEED'])

	# Generate points
	points = generatePoints(params["NUM_POINTS"],
							xRange=(0,params["SPACE_SIZE"]),
							yRange=(0,params["SPACE_SIZE"]),
							fixedZ=params["FIXED_Z"])

	# Construct and call solver
	planner = plannerFromParams(params)
	planner.solve(points)

	# Save if desired
	if "SAVE_PATH_FOLDER" in params:
		planner.printResultsToYaml()

	return planner.timeInfo

def runPlannerFromSettings(settingsFile):
	"""
	Run a planner from a settings file (or folder) path
	which may generate several sets of parameters for separate runs
	"""

	# load parameters from yaml
	params = loadYamlContents(settingsFile, planSettingsFilename)
	absFolder = toDir(settingsFile)
	params["RUN_FOLDER"] = absFolder

	# make sure independent variables are present
	independentVars = params["INDEPENDENT_VARIABLES"]
	notPresent = [k not in params for k in independentVars]
	if any(notPresent):
		print('Did not find required independent variables:\n\t',
		'\n\t'.join([independentVars[i] for i in range(len(notPresent)) if notPresent[i]]),
		'\nStopping run', sep='')
		return

	#TODO this can only find top-level variables
	#TODO allow correlated values

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
	writeYaml(independentResultsDict, os.path.join(absFolder, planTimeResultsFilename))
