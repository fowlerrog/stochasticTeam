# python imports
import sys
import os
from random import seed

# project imports
from .NodeUtils import *
from .Constants import planSettingsFilename, planTimeResultsFilename
from .RunnerUtils import loadYamlContents, appendDict, toDir, writeYaml, dictToString, getIndependentValueCombos
from .OurPlanner import *
from .OurOnlinePlanner import *
from .ClusterSolver import ClusterSolver

def plannerFromParams(params):
	plannerClass = getattr(sys.modules[__name__], params['TYPE'])
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
	points = generatePoints(params['POINTS'])

	# Construct and call solver
	validSavePath = "SAVE_PATH_FOLDER" in params and isinstance(params['SAVE_PATH_FOLDER'], str)
	savePathFolderDict = {'SAVE_PATH_FOLDER' : params['SAVE_PATH_FOLDER']} if validSavePath else {}
	planner = plannerFromParams(params['PLANNER'] | savePathFolderDict)
	planner.solve(points, startPoint = params['START_POINT'], endPoint = params['END_POINT'])

	# Save if desired
	if validSavePath:
		planner.printResultsToYaml()

	return planner.timeInfo

def runPlannerFromParamsMultiTeam(params):
	"""
	Run a planner from a parameter dict (including run folder)
	which will only produce ONE run for each of multiple teams
	"""
	validSavePath = "SAVE_PATH_FOLDER" in params and isinstance(params['SAVE_PATH_FOLDER'], str)

	# Set seed if specified
	if 'SEED' in params and params['SEED'] is not None:
		seed(params['SEED'])

	# Generate points
	points = generatePoints(params['POINTS'])

	# Distribute points between teams
	savePathFolderDict = {'SAVE_PATH_FOLDER' : params['SAVE_PATH_FOLDER']} if validSavePath else {}
	clusterSolver = ClusterSolver(params['CLUSTER'] | savePathFolderDict | {'RUN_FOLDER' : params['PLANNER']['RUN_FOLDER']})
	numTeams = params['CLUSTER']['NUM_TEAMS']
	pointsList = clusterSolver.solveClusterProblem(points,
		[params['START_POINT'] for _ in range(numTeams)],
		[params['END_POINT'] for _ in range(numTeams)]
	)
	if validSavePath:
		clusterSolver.printResultsToYaml()

	# Construct and call each solver
	timeInfo = {"CLUSTER_TIME" : clusterSolver.timeInfo["CLUSTER_TIME"]}
	for i in range(numTeams):
		savePathFolderDict = {'SAVE_PATH_FOLDER' : os.path.join(params['SAVE_PATH_FOLDER'], f'team_{i}')} if validSavePath else {}
		planner = plannerFromParams(params['PLANNER'] | savePathFolderDict)
		planner.solve(pointsList[i][1:-1], startPoint = params['START_POINT'], endPoint = params['END_POINT'])

		# Save if desired
		if validSavePath:
			planner.printResultsToYaml()

		timeInfo = appendDict(timeInfo, planner.timeInfo)

	return timeInfo

def runPlannerFromSettings(settingsFile):
	"""
	Run a planner from a settings file (or folder) path
	which may generate several sets of parameters for separate runs
	"""

	# load parameters from yaml
	params = loadYamlContents(settingsFile, planSettingsFilename)
	absFolder = toDir(settingsFile)
	params['PLANNER']["RUN_FOLDER"] = absFolder

	# validate and extract independent variables
	independentValueCombos, fullIndependentDicts = getIndependentValueCombos(params)
	if independentValueCombos is None:
		print('Failed to find independent variables')
		return

	# set up run result lists
	independentResultsDict = {}
	dependentResultsDict = {}

	# call each combination
	for i in range(len(independentValueCombos)):
		thisRunIndParams = independentValueCombos[i]
		thisRunParams = fullIndependentDicts[i]
		print('Running independent parameters:\n', thisRunIndParams, sep='')

		# set results save location, if desired
		if "SAVE_PATHS" in params and params["SAVE_PATHS"]:
			thisRunParams["SAVE_PATH_FOLDER"] = 'results_' + dictToString(thisRunIndParams)

		# run
		if 'CLUSTER' in params and params['CLUSTER'] is not None:
			thisRunOutput = runPlannerFromParamsMultiTeam(thisRunParams)
		else:
			thisRunOutput = runPlannerFromParams(thisRunParams)

		# store results
		independentResultsDict = appendDict(independentResultsDict, thisRunIndParams)
		dependentResultsDict = appendDict(dependentResultsDict, thisRunOutput)

	# write overall results
	independentResultsDict.update(dependentResultsDict) # combine
	tupleKeys = [k for k in independentResultsDict.keys() if isinstance(k, tuple)]
	for k in tupleKeys: # standardize tuples to strings
		independentResultsDict['.'.join(k)] = independentResultsDict.pop(k)
	writeYaml(independentResultsDict, os.path.join(absFolder, planTimeResultsFilename), maxDecimals=3)
