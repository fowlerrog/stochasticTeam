# python imports
import sys
import os
import traceback
import numpy as np
from multiprocessing import Pool

# project imports
from pathPlanning.Constants import planPathResultsFilename, planSettingsFilename
from pathPlanning.RunnerUtils import loadYamlContents, toDir, getVarFromString, fillIndependentVariablesFromString, removeVariableFromFolderName, sigFigs
from pathPlanning.PlannerUtils import plannerFromParams
from pathPlanning.OurPlanner import Cost

def calculatePlannedMission(planSettingsData, planResultsData, verbose=False):
	"""
	Reads a dict loaded from a plan_settings.yaml
	and a dict loaded from a plan_path_results.yaml
	and estimates plan execution time, plan air time, and plan success rate
	Note that a planSettingsData with unspecified independent variables will break this
	(this is a modified version of calculatePlannedMission from ExecuteUtils)
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
	totalAirTime = 0
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
			totalAirTime += uavMeanTime
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

	return totalMeanTime, totalAirTime, totalLogProbSuccess

def harvestMissionTimeInfo(s):
	"""
	Parallelizable function for harvesting mission time info from folder s
	returns [plannedMissionTotalTime, plannedMissionAirTime, logProbSuccess, folderName]
	"""

	plannedTotalTime = None
	plannedAirTime = None
	plannedLogProbSuccess = None
	endFolderName = None

	try:
		# load plan path results
		thisDir = toDir(s)
		planPathResults = loadYamlContents(thisDir, planPathResultsFilename, verbose=False)

		# look for path settings one folder above (this is brittle)
		planFolder = os.path.split(thisDir)[0]
		planSettings = loadYamlContents(planFolder, planSettingsFilename, verbose=False)

		# fill independent variables and get planned run time
		thisPlanSettings = fillIndependentVariablesFromString(planSettings, thisDir)
		plannedTotalTime, plannedAirTime, plannedLogProbSuccess = calculatePlannedMission(thisPlanSettings, planPathResults)

		endFolderName = os.path.split(thisDir)[1]

	except Exception:
		print('Failure during evaluation of', s)
		print(traceback.format_exc())

	return [plannedTotalTime, plannedAirTime, plannedLogProbSuccess, endFolderName]

if __name__ == '__main__':
	# print help message if necessary
	# more specifically, this script is for comparing refined and unrefined tour times (optional step 3 of the algorithm)
	if len(sys.argv) < 2 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python printMissionTimeComparison.py /path/to/results/folder/ [/path/to/results/folder2/ ...]')
		exit()

	# for each results folder, get plan_path_results time and any available execute_results times
	p = Pool()
	results = p.map(harvestMissionTimeInfo, sys.argv[1:])

	# group data together by common folder name contents
	varName = 'PLANNER.REFINE_TOURS'
	trimmedFolderNames = [(removeVariableFromFolderName(r[-1], varName), r) for r in results]
	pairs = []
	i = 0
	while i < len(trimmedFolderNames) - 1:
		j = i + 1
		found = False
		while j < len(trimmedFolderNames):
			if trimmedFolderNames[i][0] == trimmedFolderNames[j][0]:
				print('Match found for', trimmedFolderNames[i][0])
				if getVarFromString(trimmedFolderNames[i][1][-1], varName) == 'True': # this is the refined one
					pairs.append([trimmedFolderNames[i][-1], trimmedFolderNames[j][-1]])
				else:
					pairs.append([trimmedFolderNames[j][-1], trimmedFolderNames[i][-1]])
				trimmedFolderNames.pop(max(i,j))
				trimmedFolderNames.pop(min(i,j))
				found = True
				break
			j += 1
		if not found:
			print('NO MATCH FOUND FOR', trimmedFolderNames[i][0])
			i += 1
	if len(pairs) == 0:
		print('No refined/unrefined pairs found')
		exit()

	# evaluate ratios, grouped by # of points
	nVarName = 'POINTS.NUM_POINTS'
	nSigFig = 4
	numPoints = {int(getVarFromString(r[-1], nVarName)) for r in results}
	sortedNumPoints = sorted(list(numPoints))
	print('ratios are: 1 - (refined/unrefined) reported as (mean,sigma) and [min,max]')
	print('n\tair time\ttotal time\tp_risk')
	for n in sortedNumPoints:
		thisPairs = [p for p in pairs if int(getVarFromString(p[0][-1], nVarName)) == n]
		airRatios = [1 - p[0][1] / p[1][1] for p in thisPairs]
		totalRatios = [1 - p[0][0] / p[1][0] for p in thisPairs]
		# riskRatios = [(1 - safeExp(p[0][2])) / (1 - safeExp(p[1][2])) for p in thisPairs] # numerical issue as p[1][2] -> 1
		# replace w/ (exp(p1)-1) / (exp(p2)-1)
		# and also do 1 - ratio for consistency
		riskRatios = [1 - np.expm1(p[0][2]) / np.expm1(p[1][2]) for p in thisPairs]
		# print('\n'.join(str(p) for p in thisPairs))
		print(f'{n}\t' +
		f'({sigFigs(np.nanmean(airRatios), nSigFig)}, {sigFigs(np.nanstd(airRatios), nSigFig)})\t' +
		f'({sigFigs(np.nanmean(totalRatios), nSigFig)}, {sigFigs(np.nanstd(totalRatios), nSigFig)})\t' +
		f'({sigFigs(np.nanmean(riskRatios), nSigFig)}, {sigFigs(np.nanstd(riskRatios), nSigFig)})\n\t' +
		f'[{sigFigs(np.nanmin(airRatios), nSigFig)}, {sigFigs(np.nanmax(airRatios), nSigFig)}]\t' +
		f'[{sigFigs(np.nanmin(totalRatios), nSigFig)}, {sigFigs(np.nanmax(totalRatios), nSigFig)}]\t' +
		f'[{sigFigs(np.nanmin(riskRatios), nSigFig)}, {sigFigs(np.nanmax(riskRatios), nSigFig)}]'
		)
