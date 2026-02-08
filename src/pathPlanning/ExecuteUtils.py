# python imports
import os
from random import seed
from copy import deepcopy

# project imports
from .Constants import executeSettingsFilename, planSettingsFilename, planPathResultsFilename, executeResultsFilename
from .RunnerUtils import writeYaml, loadYamlContents, toDir, dictToString, getIndependentValueCombos
from .EnvUtils import envFromParams
from .PlannerUtils import plannerFromParams
from .OurPlanner import Cost

def executePlanFromParams(executeParams, planSettingsPath, planResultsPath):
	"""
	Executes a number of runs for a given plan
	"""

	# load parameters from yamls
	absPlanSettingsPath = os.path.abspath(planSettingsPath)
	planParams = loadYamlContents(absPlanSettingsPath, planSettingsFilename)
	absResultsPath = os.path.abspath(planResultsPath)
	resultsDict = loadYamlContents(absResultsPath, planPathResultsFilename)

	# construct environment
	envParams = executeParams['ENVIRONMENT']
	env = envFromParams(envParams)

	# parse path
	uavPoints = resultsDict['uav_points']
	uavTours = resultsDict['uav_tours']
	ugvOrder = resultsDict['ugv_path']
	ugvPoints = [[*v, 0] for v in resultsDict['ugv_point_map']] # 2d -> 3d

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

		# travel to first release
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
				env.evaluate(uavPoints[thisUavTour[len(thisUavTour)-1]], collectPoint, 'UAV')

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
			if iTour < len(uavTours) - 1:
				chargeTime = (uavMaxTime - uavTime) / uavChargeRate
				ugvTransitTimesThisRun.append( max(
					chargeTime,
					env.evaluate(
						collectPoint,
						ugvPoints[ugvOrder[3 + 2*iTour]],
						'UGV'
					)
				) )

		# travel to end point
		ugvTransitTimesThisRun.append(
			env.evaluate(
				ugvPoints[ugvOrder[-2]],
				ugvPoints[ugvOrder[-1]],
				'UGV'
			)
		)

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

def executePlanFromParamsWithOnlinePlanner(executeParams, planSettingsPath, planResultsPath, verbose=False):
	"""
	Executes a number of runs for a given plan
	using an online planner to replan every so often
	"""

	# load parameters from yamls
	absPlanSettingsPath = os.path.abspath(planSettingsPath)
	planParams = loadYamlContents(absPlanSettingsPath, planSettingsFilename)
	absResultsPath = os.path.abspath(planResultsPath)
	resultsDict = loadYamlContents(absResultsPath, planPathResultsFilename)

	# construct environment
	envParams = executeParams['ENVIRONMENT']
	if verbose:
		print("Constructing environment:", envParams)
	env = envFromParams(envParams)

	# construct online planner
	onlinePlanner = None
	if 'ONLINE_PLANNER' in executeParams:
		onlineParams = executeParams['ONLINE_PLANNER']
		if verbose:
			print("Constructing online planner:", onlineParams)
		onlinePlanner = plannerFromParams(onlineParams)

	# parse path
	uavPoints = resultsDict['uav_points'] # 3d
	uavTours = resultsDict['uav_tours']
	ugvOrder = resultsDict['ugv_path']
	ugvPoints = resultsDict['ugv_point_map'] # 2d

	# TODO perhaps this should be in its own agent definition file, or the environment?
	uavMaxTime = planParams['PLANNER']['UAV_BATTERY_TIME']
	uavChargeRate = planParams['PLANNER']['CHARGE_RATE']

	# set up RNG
	if 'SEED' in executeParams and executeParams['SEED'] is not None:
		seed(executeParams['SEED'])

	# execute runs
	tourAttempts = 0 # how many total tour attempts were there
	timeoutFailures = 0 # how many tours failed
	uavFinalTours = [] # list of actual uav tours
	ugvFinalOrders = [] # list of actual ugv orders
	ugvFinalPoints = [] # list of actual ugv points
	tourTimes = [] # list of lists of tour times
	totalTimes = [] # list of total plan execution times
	replanTimes = [] # list of list of replan computation times
	replanFailures = 0 # how many total replan failures were there

	numRuns = executeParams['NUM_RUNS']
	print('Running %d runs'%numRuns)
	for iRun in range(numRuns):
		if verbose:
			print(f'Starting run {iRun}')

		thisUavTours = deepcopy(uavTours)
		thisUgvOrder = deepcopy(ugvOrder)
		thisUgvPoints = deepcopy(ugvPoints)

		ugvIndex = 0 # most recent ugvOrder position
		ugvPosition = None # actual ugv 3d position
		iTour = 0 # most recent position in uavTours, or the upcoming tour if between tours
		jTour = 0 # most recent position in uavTours[i], or the upcoming tour if between tours
		uavTourTime = 0 # how long the UAV has been in flight (depleting its battery)

		thisTourTimes = []
		thisTotalTime = 0
		thisReplanTimes = []

		# note that every instance of this loop occurs just after we are AT ugvIndex, iTour, jTour
		#	and that we cannot change the plan for those points because they have occurred
		while ugvIndex < len(thisUgvOrder) - 1:
			thisUavTours, thisUgvOrder, thisUgvPoints, \
			ugvIndex, ugvPosition, iTour, jTour, uavTourTime, \
			thisTourTimes, thisTotalTime, thisReplanTimes, \
			timeoutFailures, replanFailures, tourAttempts = \
			stepOnlineExecution(onlinePlanner, env, uavPoints,
				thisUavTours, thisUgvOrder, thisUgvPoints,
				ugvIndex, ugvPosition, iTour, jTour, uavTourTime,
				thisTourTimes, thisTotalTime, thisReplanTimes,
				timeoutFailures, replanFailures, tourAttempts,
				uavChargeRate, uavMaxTime,
				verbose=verbose
			)

		# store results at end of plan
		uavFinalTours.append(thisUavTours)
		ugvFinalOrders.append(thisUgvOrder)
		ugvFinalPoints.append(thisUgvPoints)
		totalTimes.append(thisTotalTime)
		tourTimes.append(thisTourTimes)
		replanTimes.append(thisReplanTimes)

	# write results
	results = {
		'NUM_RUNS': numRuns,
		'TOUR_ATTEMPTS': tourAttempts,
		'UAV_FINAL_TOURS': uavFinalTours,
		'UGV_FINAL_ORDERS': ugvFinalOrders,
		'UGV_FINAL_POINTS': ugvFinalPoints,
		'TOTAL_TIMES': totalTimes,
		'TOUR_TIMES': tourTimes,
		'REPLAN_TIMES': replanTimes,
		'REPLAN_FAILURES': replanFailures
	}
	return results

def stepOnlineExecution(onlinePlanner, env, uavPoints,
				thisUavTours, thisUgvOrder, thisUgvPoints,
				ugvIndex, ugvPosition, iTour, jTour, uavTourTime,
				thisTourTimes, thisTotalTime, thisReplanTimes,
				timeoutFailures, replanFailures, tourAttempts,
				uavChargeRate, uavMaxTime,
				verbose=False):
	"""Executes one UAV point in an online-replanning scenario"""
	while True: # this is a hack so I can break
		# ended final tour
		if ugvIndex == len(thisUgvOrder) - 2:
			# do transit without charging
			ugvTransitTime = env.evaluate(
				thisUgvPoints[thisUgvOrder[ugvIndex]],
				thisUgvPoints[thisUgvOrder[ugvIndex+1]],
				'UGV'
			)
			thisTotalTime += ugvTransitTime
			if verbose:
				print(f'Transiting to end : {thisUgvPoints[thisUgvOrder[ugvIndex]]} -> {thisUgvPoints[thisUgvOrder[ugvIndex+1]]} = {ugvTransitTime:.2f}')
			ugvIndex += 1
			break

		# we are between tours, just after collect (or before first tour)
		elif ugvIndex % 2 == 0:
			# replan, agents together
			if onlinePlanner is not None:
				uavPos = onlinePlanner.project(thisUgvPoints[thisUgvOrder[ugvIndex]])
				ugvPos = thisUgvPoints[thisUgvOrder[ugvIndex]]
				if verbose:
					print(f'Replanning at UAV {iTour} {jTour} = {uavPos}\tUGV {ugvIndex} = [' + ', '.join(f'{u}' for u in ugvPos) + ']')
				thisUavTours, thisUgvOrder, thisUgvPoints, successFlag = onlinePlanner.solve(
					thisUavTours, uavPoints,
					thisUgvOrder, thisUgvPoints,
					iTour, jTour, ugvIndex,
					uavPos, ugvPos,
					0
				)
				thisReplanTimes.append(onlinePlanner.getSolveTime())
				replanFailures += not successFlag
				if thisUavTours is None:
					print('No valid plan found')
					break
			# do transit
			chargeTime = uavTourTime / uavChargeRate
			ugvTransitTime = env.evaluate(
				thisUgvPoints[thisUgvOrder[ugvIndex]],
				thisUgvPoints[thisUgvOrder[ugvIndex+1]],
				'UGV'
			)
			if verbose:
				print(f'Transiting to tour {ugvIndex // 2} : {thisUgvPoints[thisUgvOrder[ugvIndex]]} -> {thisUgvPoints[thisUgvOrder[ugvIndex+1]]} = {ugvTransitTime:.2f} w/ charge = {chargeTime:.2f}')
			thisTotalTime += max(chargeTime, ugvTransitTime)
			ugvIndex += 1
			ugvPosition = thisUgvPoints[thisUgvOrder[ugvIndex]]

		# we are in a tour
		else:
			deltaT = 0 # time change during this loop instance
			# we are starting a tour
			if jTour == 0:
				tourAttempts += 1
				# take off
				uavTourTime = 0
				takeoffTime = env.evaluate(onlinePlanner.project(ugvPosition), uavPoints[thisUavTours[iTour][jTour]], 'UAV')
				deltaT += takeoffTime
				if verbose:
					print(f'Starting tour {iTour} : {ugvPosition} -> {uavPoints[thisUavTours[iTour][jTour]]} = {takeoffTime:.2f}')

			# replan, agents apart
			if onlinePlanner is not None:
				uavPos = uavPoints[thisUavTours[iTour][jTour]]
				if verbose:
					print(f'Replanning at UAV {iTour} {jTour} = {uavPos}\tUGV {ugvIndex} = [' + ', '.join(f'{u}' for u in ugvPosition) + f']\tt = {uavTourTime:2f}')
				thisUavTours, thisUgvOrder, thisUgvPoints, successFlag = onlinePlanner.solve(
					thisUavTours, uavPoints,
					thisUgvOrder, thisUgvPoints,
					iTour, jTour, ugvIndex,
					uavPos, ugvPosition,
					uavTourTime
				)
				thisReplanTimes.append(onlinePlanner.getSolveTime())
				replanFailures += not successFlag
				if thisUavTours is None:
					print('No valid plan found')
			collectPoint = thisUgvPoints[thisUgvOrder[ugvIndex+1]]

			# normal part of UAV movement to next air point
			airTime = env.evaluate(uavPoints[thisUavTours[iTour][jTour]], uavPoints[thisUavTours[iTour][jTour+1]], 'UAV')
			deltaT += airTime
			if verbose:
				print(f'Taking tour {iTour} step {jTour} : {uavPoints[thisUavTours[iTour][jTour]]} -> {uavPoints[thisUavTours[iTour][jTour+1]]} = {airTime:.2f}')

			# we are ending a tour
			finishedTour = False
			if jTour == len(thisUavTours[iTour]) - 2:
				finishedTour = True
				# land
				landingTime = env.evaluate(uavPoints[thisUavTours[iTour][jTour+1]], onlinePlanner.project(collectPoint), 'UAV')
				if verbose:
					print(f'Ending tour {iTour} : {uavPoints[thisUavTours[iTour][jTour+1]]} -> {collectPoint} = {landingTime:.2f}')
				deltaT += landingTime

			# update uav time
			uavTourTime += deltaT

			# UGV has not arrived yet
			if ugvPosition != collectPoint:
				# update UGV position
				#	we know deltaT has elapsed, and we can evaluate UGV travel time
				actualUgvTime = env.evaluate(ugvPosition, collectPoint, 'UGV')
				ugvOldPosition = ugvPosition
				if actualUgvTime <= deltaT:
					ugvPosition = collectPoint
				else:
					ugvPosition = [ugvPosition[i] + (collectPoint[i] - ugvPosition[i]) * deltaT / actualUgvTime for i in range(2)]
				if verbose:
					print(f'UGV movement : [' + ', '.join([f'{p:.2f}' for p in ugvOldPosition]) + f'] -> [' + ', '.join([f'{p:.2f}' for p in ugvPosition]) + f']')
			elif verbose:
				print('UGV already arrived')

			# UGV must still move to finish the tour
			if finishedTour and ugvPosition != collectPoint:
				ugvFinishTime = env.evaluate(ugvPosition, collectPoint, 'UGV')
				if verbose:
					print(f'UGV additional time : [' + ', '.join([f'{p:.2f}' for p in ugvPosition]) + f'] -> {collectPoint} = {ugvFinishTime:.2f}')
				ugvPosition = collectPoint
				uavTourTime += ugvFinishTime
				if verbose:
					print(f'Final tour time : {uavTourTime:.2f}')

			# check for failure
			if uavTourTime >= uavMaxTime:
				timeoutFailures += 1
				thisTotalTime += uavTourTime
				thisTourTimes.append(uavTourTime)
				if verbose:
					print(f'FAILURE')
				break

			jTour += 1

			# check for end of tour
			if finishedTour:
				jTour = 0
				ugvIndex += 1
				thisTotalTime += uavTourTime
				thisTourTimes.append(uavTourTime)
				if verbose:
					print(f'Successfully finished tour {iTour}')
				iTour += 1
		
		break # only do loop once
	
	return thisUavTours, thisUgvOrder, thisUgvPoints, \
	ugvIndex, ugvPosition, iTour, jTour, uavTourTime, \
	thisTourTimes, thisTotalTime, thisReplanTimes, \
	timeoutFailures, replanFailures, tourAttempts

def executePlanFromSettings(executeSettingsPath, planSettingsPath, planResultsPath):
	"""
	Executes a plan from a settings file (or folder) path
	which may generate several sets of parameters for separate runs
	"""

	# load parameters from yaml
	absExecutePath = os.path.abspath(executeSettingsPath)
	executeParams = loadYamlContents(absExecutePath, executeSettingsFilename)

	# validate and extract independent variables
	independentValueCombos, fullIndependentDicts = getIndependentValueCombos(executeParams)
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
		if 'ONLINE_PLANNER' in thisRunParams:
			thisRunOutput = executePlanFromParamsWithOnlinePlanner(thisRunParams, planSettingsPath, planResultsPath, verbose=True)
		else:
			thisRunOutput = executePlanFromParams(thisRunParams, planSettingsPath, planResultsPath)

		# write results
		yamlOutPath = os.path.join(absResultsFolder, executeResultsFilename)
		if len(thisRunIndParams) > 0:
			yamlOutPathParts = os.path.splitext(yamlOutPath)
			indVarString = dictToString(thisRunIndParams)
			yamlOutPath = yamlOutPathParts[0] + '_' + indVarString + yamlOutPathParts[1]
		writeYaml(thisRunOutput, yamlOutPath, maxDecimals=1)

def calculatePlannedMission(planSettingsData, planResultsData, verbose=False):
	"""
	Reads a dict loaded from a plan_settings.yaml
	and a dict loaded from a plan_path_results.yaml
	and estimates plan execution time and log prob success
	Note that a planSettingsData with unspecified independent variables will break this
	"""

	# construct planner
	planner = plannerFromParams(planSettingsData['PLANNER'])

	# parse path
	uavPoints = planResultsData['uav_points']
	uavTours = planResultsData['uav_tours']
	ugvOrder = planResultsData['ugv_path']
	ugvPoints = [[*v, 0] for v in planResultsData['ugv_point_map']] # 2d -> 3d

	uavChargeRate = planSettingsData['PLANNER']['CHARGE_RATE']

	# sum over tours
	totalMeanTime = 0
	totalLogProbSuccess = 0

	# travel to first release
	totalMeanTime += planner.env.estimateMean(
		ugvPoints[ugvOrder[0]],
		ugvPoints[ugvOrder[1]],
		'UGV'
	)
	if verbose:
		print(f'UGV transiting to first release : {ugvPoints[ugvOrder[0]]} -> {ugvPoints[ugvOrder[1]]} = {totalMeanTime:.2f}')

	for iTour in range(len(uavTours)):
			# calculate ugv travel time and prob success
			releasePoint = ugvPoints[ugvOrder[1 + 2*iTour]]
			collectPoint = ugvPoints[ugvOrder[2 + 2*iTour]]
			ugvMeanTime = planner.env.estimateMean(releasePoint, collectPoint, 'UGV')
			ugvVarTime = planner.env.estimateVariance(releasePoint, collectPoint, 'UGV')
			ugvLogProbSuccess = planner.evaluateConstraintFloat(Cost(ugvMeanTime, ugvVarTime), 'UGV')
			if verbose:
				print(f'UGV tour time : {releasePoint} -> {collectPoint} = {ugvMeanTime:.2f} {ugvVarTime:.2f}')

			# calculate uav travel time
			thisUavTour = uavTours[iTour]
			uavMeanTime = \
				planner.env.estimateMean(releasePoint, uavPoints[thisUavTour[0]], 'UAV') + \
				sum([
					planner.env.estimateMean(uavPoints[thisUavTour[j]], uavPoints[thisUavTour[j+1]], 'UAV')
					for j in range(0, len(thisUavTour) - 1)
				]) + \
				planner.env.estimateMean(uavPoints[thisUavTour[len(thisUavTour)-1]], collectPoint, 'UAV')
			uavVarTime = \
				planner.env.estimateVariance(releasePoint, uavPoints[thisUavTour[0]], 'UAV') + \
				sum([
					planner.env.estimateVariance(uavPoints[thisUavTour[j]], uavPoints[thisUavTour[j+1]], 'UAV')
					for j in range(0, len(thisUavTour) - 1)
				]) + \
				planner.env.estimateVariance(uavPoints[thisUavTour[len(thisUavTour)-1]], collectPoint, 'UAV')
			uavLogProbSuccess = planner.evaluateConstraintFloat(Cost(uavMeanTime, uavVarTime), 'UAV')
			if verbose:
				print(f'UAV tour time : {releasePoint} -> ... -> {collectPoint} = {uavMeanTime:.2f} {uavVarTime:.2f}')

			# store time
			totalMeanTime += max(uavMeanTime, ugvMeanTime)
			totalLogProbSuccess += uavLogProbSuccess + ugvLogProbSuccess

			# calculate ugv time to next release
			if iTour < len(uavTours) - 1:
				chargeTime = max(uavMeanTime, ugvMeanTime) / uavChargeRate
				ugvMeanTransitTime = planner.env.estimateMean(
					collectPoint,
					ugvPoints[ugvOrder[3 + 2*iTour]],
					'UGV'
				)
				totalMeanTime += max(chargeTime, ugvMeanTransitTime)
				if verbose:
					print(f'UGV intertour time : {collectPoint} -> {ugvPoints[ugvOrder[3 + 2*iTour]]} = {ugvMeanTransitTime:.2f} w/ charge = {chargeTime:.2f}')

	# travel to end point
	ugvFinishTime = planner.env.estimateMean(
		ugvPoints[ugvOrder[-2]],
		ugvPoints[ugvOrder[-1]],
		'UGV'
	)
	totalMeanTime += ugvFinishTime
	if verbose:
		print(f'UGV transiting to final point : {ugvPoints[ugvOrder[-2]]} -> {ugvPoints[ugvOrder[-1]]} = {ugvFinishTime:.2f}')

	return totalMeanTime, totalLogProbSuccess
