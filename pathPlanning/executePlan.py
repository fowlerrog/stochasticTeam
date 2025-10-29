# python imports
import sys
import os
import traceback
from random import seed

# project imports
from pathPlanning.Constants import executeSettingsFilename, planSettingsFilename, planPathResultsFilename, executeResultsFilename
from pathPlanning.RunnerUtils import writeYaml, loadYamlContents, toDir
from pathPlanning.EnvUtils import envFromParamsOrFile

def executePlanFromSettings(executeSettingsPath, planSettingsPath, planResultsPath):
	"""Executes a number of runs for a given plan"""

	# load parameters from yamls
	absExecutePath = os.path.abspath(executeSettingsPath)
	executeParams = loadYamlContents(absExecutePath, executeSettingsFilename)
	absPlanSettingsPath = os.path.abspath(planSettingsPath)
	planParams = loadYamlContents(absPlanSettingsPath, planSettingsFilename)
	absResultsPath = os.path.abspath(planResultsPath)
	resultsDict = loadYamlContents(absResultsPath, planPathResultsFilename)

	# construct environment
	envParams = executeParams['ENVIRONMENT']
	if isinstance(envParams, str): # this is a path to another file, not params
		envParams = os.path.join(toDir(absExecutePath), envParams)
	env = envFromParamsOrFile(envParams)

	# parse path
	uavPoints = resultsDict['uav_points']
	uavCycles = resultsDict['uav_cycles']
	ugvOrder = resultsDict['ugv_path']
	ugvPoints = resultsDict['ugv_mapping_to_points']
	ugvPoints = {int(k):[*v, 0] for k,v in ugvPoints.items()} # str -> int : 2d -> 3d

	# TODO perhaps this should be in its own agent definition file, or the environment?
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
			# print('icycle', iCycle)
			# calculate ugv travel time
			releasePoint = ugvPoints[ugvOrder[1 + 2*iCycle]]
			collectPoint = ugvPoints[ugvOrder[2 + 2*iCycle]]
			ugvTime = env.evaluate(releasePoint, collectPoint, 'UGV')
			# print('ugv', releasePoint, '->', collectPoint, '=', ugvTime)

			# calculate uav travel time
			thisUavCycle = uavCycles[iCycle]
			uavTime = \
				env.evaluate(releasePoint, uavPoints[thisUavCycle[0]], 'UAV') + \
				sum([
					env.evaluate(uavPoints[thisUavCycle[j]], uavPoints[thisUavCycle[j+1]], 'UAV')
					for j in range(0, len(thisUavCycle) - 1)
				]) + \
				env.evaluate(uavPoints[thisUavCycle[len(thisUavCycle)-1]], collectPoint, 'UAV') # TODO projection?
			# print('uav\n',
			# 	releasePoint, '->', uavPoints[thisUavCycle[0]], '=',
			# 	env.evaluate(releasePoint, uavPoints[thisUavCycle[0]], 'UAV'), '\n',
			# 	'\n'.join([
			# 		str(uavPoints[thisUavCycle[j]]) + ' -> ' +
			# 		str(uavPoints[thisUavCycle[j+1]]) + ' = ' +
			# 		str(env.evaluate(uavPoints[thisUavCycle[j]], uavPoints[thisUavCycle[j+1]], 'UAV') ) for j in range(0, len(thisUavCycle) - 1)
			# 	]), '\n',
			# 	uavPoints[thisUavCycle[len(thisUavCycle)-1]], '->', collectPoint, '=',
			# 	env.evaluate(uavPoints[thisUavCycle[len(thisUavCycle)-1]], collectPoint, 'UAV'), '\n=',
			# 	uavTime)

			# TODO hovering - we are just comparing max flight time to UGV time. is that sufficient?

			# evaluate success / failure
			failure = False
			remainingFlightTimesThisRun.append(uavMaxTime - max(uavTime, ugvTime))
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
	writeYaml(results, os.path.join(planFolder, executeResultsFilename))

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
