# python imports
import sys
import os
import traceback
from random import seed

# project imports
from MultiAgentTypeEnvironment import *
from Constants import executeSettingsFilename, planSettingsFilename, planPathResultsFilename, executeResultsFilename
from RunnerUtils import *

def executePlanFromSettings(executeSettingsPath, planSettingsPath, planResultsPath):
	"""Executes a number of runs for a given plan"""

	# load parameters from jsons
	absExecutePath = os.path.abspath(executeSettingsPath)
	executeParams = loadJsonContents(absExecutePath, executeSettingsFilename)
	absPlanSettingsPath = os.path.abspath(planSettingsPath)
	planParams = loadJsonContents(absPlanSettingsPath, planSettingsFilename)
	absResultsPath = os.path.abspath(planResultsPath)
	resultsDict = loadJsonContents(absResultsPath, planPathResultsFilename)

	# construct environment
	executeFolder = toDir(absExecutePath)
	envPath = os.path.join(executeFolder, executeParams['ENVIRONMENT'])
	envParams = loadJsonContents(envPath)
	envClass = getattr(sys.modules[__name__], envParams['TYPE'])
	env = envClass(envParams)

	# parse path
	uavPoints = resultsDict['tsp_points']
	uavCycles = resultsDict['cycles']
	ugvOrder = resultsDict['path']
	ugvPoints = resultsDict['mapping_to_points']
	ugvPoints = {int(k):[*v, 0] for k,v in ugvPoints.items()} # str -> int : 2d -> 3d

	# TODO perhaps this should be in its own agent definition file, or the environment?
	uavExtraTime = planParams['TAKEOFF_LANDING_TIME']
	uavMaxTime = planParams['UAV_BATTERY_TIME']

	# set up RNG
	if 'SEED' in executeParams and executeParams['SEED'] is not None:
		seed(executeParams['SEED'])

	# execute runs
	uavTimeoutFailures = 0
	ugvTimeoutFailures = 0
	remainingFlightTimes = []
	numRuns = executeParams['NUM_RUNS']
	print('Running %d runs'%numRuns)
	for _ in range(numRuns):
		remainingFlightTimesThisRun = []
		for iCycle in range(len(uavCycles)):
			# calculate ugv travel time
			releasePoint = ugvPoints[ugvOrder[1 + 2*iCycle]]
			collectPoint = ugvPoints[ugvOrder[2 + 2*iCycle]]
			ugvTime = env.evaluate(releasePoint, collectPoint, 'UGV')

			# calculate uav travel time
			thisUavCycle = uavCycles[iCycle]
			uavTime = uavExtraTime + \
				sum([
				env.evaluate(uavPoints[thisUavCycle[j]], uavPoints[thisUavCycle[j+1]], 'UAV')
				for j in range(len(thisUavCycle) - 1)
				]) + \
				env.evaluate(uavPoints[thisUavCycle[len(thisUavCycle)-1]], collectPoint, 'UAV') # TODO projection?

			# evaluate success / failure
			failure = False
			remainingFlightTimesThisRun.append(uavMaxTime - uavTime)
			# TODO should we consider ugv time for the above?
			if ugvTime > uavMaxTime:
				ugvTimeoutFailures += 1
				failure = True
			if uavTime > uavMaxTime:
				uavTimeoutFailures += 1
				failure = True

			if failure:
				break
		remainingFlightTimes.append(remainingFlightTimesThisRun)

	# write results
	results = {
		'NUM_RUNS': numRuns,
		'UAV_TIMEOUT_FAILURES':	uavTimeoutFailures,
		'UGV_TIMEOUT_FAILURES': ugvTimeoutFailures,
		'REMAINING_FLIGHT_TIMES': remainingFlightTimes
	}
	planFolder = toDir(absResultsPath)
	writeJson(results, os.path.join(planFolder, executeResultsFilename))

### main function
if __name__ == '__main__':
	# print help message if necessary
	if len(sys.argv) < 4 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python executePlan.py /path/to/%s /path/to/%s /path/to/%s [/path/to/another/%s ...]'%(executeSettingsFilename, planSettingsFilename, planPathResultsFilename, planPathResultsFilename))
		exit()
	
	# for each provided settings file, run planner
	for s in sys.argv[3:]:
		try:
			executePlanFromSettings(sys.argv[1], sys.argv[2], s)
		except Exception:
			print(traceback.format_exc())
