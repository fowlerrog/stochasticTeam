# python imports
import os
from random import seed
from copy import deepcopy

# project imports
from .Constants import executeSettingsFilename, planSettingsFilename, planPathResultsFilename, executeResultsFilename
from .RunnerUtils import writeYaml, loadYamlContents, toDir, dictToString, getIndependentValueCombos
from .EnvUtils import envFromParams

def executePlanFromParams(executeParams, executeSettingsPath, planSettingsPath, planResultsPath):
	"""Executes a number of runs for a given plan"""

	# load parameters from yamls
	absExecutePath = os.path.abspath(executeSettingsPath)
	absPlanSettingsPath = os.path.abspath(planSettingsPath)
	planParams = loadYamlContents(absPlanSettingsPath, planSettingsFilename)
	absResultsPath = os.path.abspath(planResultsPath)
	resultsDict = loadYamlContents(absResultsPath, planPathResultsFilename)

	# construct environment
	envParams = executeParams['ENVIRONMENT']
	env = envFromParams(envParams)

	# construct online planner
	onlinePlanner = None
	if 'ONLINE_PLANNER' in executeParams:
		pass

	# parse path
	uavPoints = resultsDict['uav_points']
	uavTours = resultsDict['uav_tours']
	ugvOrder = resultsDict['ugv_path']
	ugvPoints = resultsDict['ugv_point_map']
	ugvPoints = {int(k):[*v, 0] for k,v in ugvPoints.items()} # str -> int : 2d -> 3d

	# TODO perhaps this should be in its own agent definition file, or the environment?
	uavMaxTime = planParams['PLANNER']['UAV_BATTERY_TIME']
	uavChargeRate = planParams['PLANNER']['CHARGE_RATE']

	# set up RNG
	if 'SEED' in executeParams and executeParams['SEED'] is not None:
		seed(executeParams['SEED'])

	# execute runs
	tourAttempts = [0] * len(uavTours)
	uavTimeoutFailures = [0] * len(uavTours)
	ugvTimeoutFailures = [0] * len(uavTours)
	remainingFlightTimes = []
	ugvTransitTimes = []
	numRuns = executeParams['NUM_RUNS']
	print('Running %d runs'%numRuns)
	for _ in range(numRuns):
		remainingFlightTimesThisRun = []
		ugvTransitTimesThisRun = [
			env.evaluate(
				ugvPoints[ugvOrder[0]],
				ugvPoints[ugvOrder[1]],
				'UGV'
			)
		]
		for iTour in range(len(uavTours)):
			tourAttempts[iTour] += 1
			# calculate ugv travel time
			releasePoint = ugvPoints[ugvOrder[1 + 2*iTour]]
			collectPoint = ugvPoints[ugvOrder[2 + 2*iTour]]
			ugvTime = env.evaluate(releasePoint, collectPoint, 'UGV')

			# calculate uav travel time
			thisUavTour = uavTours[iTour]
			uavTime = \
				env.evaluate(releasePoint, uavPoints[thisUavTour[0]], 'UAV') + \
				sum([
					env.evaluate(uavPoints[thisUavTour[j]], uavPoints[thisUavTour[j+1]], 'UAV')
					for j in range(0, len(thisUavTour) - 1)
				]) + \
				env.evaluate(uavPoints[thisUavTour[len(thisUavTour)-1]], collectPoint, 'UAV') # TODO projection?

			# evaluate success / failure
			failure = False
			remainingFlightTimesThisRun.append(uavMaxTime - max(uavTime, ugvTime))
			if ugvTime > uavMaxTime:
				ugvTimeoutFailures[iTour] += 1
				failure = True
			if uavTime > uavMaxTime:
				uavTimeoutFailures[iTour] += 1
				failure = True

			if failure:
				break

			# calculate ugv time to next release
			#	note that this includes waiting at the final end point for 100% charge
			chargeTime = (uavMaxTime - uavTime) / uavChargeRate
			ugvTransitTimesThisRun.append( max(
				chargeTime,
				env.evaluate(
					collectPoint,
					ugvPoints[ugvOrder[3 + 2*iTour]],
					'UGV'
				)
			) )

		remainingFlightTimes.append(remainingFlightTimesThisRun)
		ugvTransitTimes.append(ugvTransitTimesThisRun)

	# write results
	results = {
		'NUM_RUNS': numRuns,
		'TOUR_ATTEMPTS': tourAttempts,
		'UAV_TIMEOUT_FAILURES':	uavTimeoutFailures,
		'UGV_TIMEOUT_FAILURES': ugvTimeoutFailures,
		'REMAINING_FLIGHT_TIMES': remainingFlightTimes,
		'UGV_TRANSIT_TIMES': ugvTransitTimes
	}
	return results

def executePlanFromSettings(executeSettingsPath, planSettingsPath, planResultsPath):
	"""
	Executes a plan from a settings file (or folder) path
	which may generate several sets of parameters for separate runs
	"""

	# load parameters from yaml
	absExecutePath = os.path.abspath(executeSettingsPath)
	params = loadYamlContents(absExecutePath, executeSettingsFilename)

	# validate and extract independent variables
	independentValueCombos, fullIndependentDicts = getIndependentValueCombos(params)
	if independentValueCombos is None:
		print('Failed to find independent variables')
		return

	# call each combination
	absResultsFolder = toDir(os.path.abspath(planResultsPath))
	for i in range(len(independentValueCombos)):
		thisRunIndParams = independentValueCombos[i]
		thisRunParams = fullIndependentDicts[i]
		print('Running independent parameters:\n', thisRunIndParams, sep='')

		# run
		thisRunOutput = executePlanFromParams(thisRunParams, executeSettingsPath, planSettingsPath, planResultsPath)

		# write results
		yamlOutPath = os.path.join(absResultsFolder, executeResultsFilename)
		if len(thisRunIndParams) > 0:
			yamlOutPathParts = os.path.splitext(yamlOutPath)
			indVarString = dictToString(thisRunIndParams)
			yamlOutPath = yamlOutPathParts[0] + '_' + indVarString + yamlOutPathParts[1]
		writeYaml(thisRunOutput, yamlOutPath, maxDecimals=1)
