# python imports
import sys
import matplotlib.pyplot as plt
import os
from scipy.spatial.distance import euclidean

# project imports
from pathPlanning.ExecuteUtils import calculatePlannedMission
from pathPlanning.RunnerUtils import loadYamlContents, fillIndependentVariablesFromString
from pathPlanning.Constants import planSettingsFilename, planPathResultsFilename, executeResultsFilename
from pathPlanning.OurPlanner import safeExp
from pathPlanning.PlotUtils import plotSelfComparison

if __name__ == '__main__':
	# print help message if necessary
	if len(sys.argv) < 2 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python plotOnlineMissionTimes.py /path/to/results/folder/ [/path/to/results/folder2/ ...]')
		exit()

	folderNames = sys.argv[1:]
	distTol = 1e0

	plannedTimes = [] # total planned mean times
	plannedSuccessRates = [] # total planned success rates
	realSuccessfulTimes = [] # mean execution times of successful plans
	realSuccessRates = [] # mean success rates
	planChangeRates = [] # how many runs did we have any replans, out of total runs

	for f in folderNames:
		# look for path settings one folder above (this is brittle)
		planFolder = os.path.split(f)[0]
		planSettingsData = loadYamlContents(planFolder, planSettingsFilename)

		# choose independent plan variables
		thisPlanSettingsData = fillIndependentVariablesFromString(planSettingsData, f)

		# load plan results
		planResultsData = loadYamlContents(f, planPathResultsFilename)
		plannedTime, plannedLogSuccessRate = calculatePlannedMission(thisPlanSettingsData, planResultsData)
		plannedTimes.append(plannedTime)
		plannedSuccessRates.append(safeExp(plannedLogSuccessRate))

		# load execute results
		executeResults = loadYamlContents(f, executeResultsFilename)
		if len(executeResults) == 0:
			print(f'Skipping empty or nonexistent {os.path.join(f, executeResultsFilename)}')
			plannedTimes.pop()
			plannedSuccessRates.pop()
			continue

		totalRealTime = 0
		numFailures = 0
		numReplans = 0
		for i in range(len(executeResults['TOTAL_TIMES'])):
			# look for failure
			if executeResults['TOUR_TIMES'][i][-1] >= thisPlanSettingsData['PLANNER']['UAV_BATTERY_TIME']:
				numFailures += 1
			else:
				totalRealTime += executeResults['TOTAL_TIMES'][i]

			# look for replans
			if any(len(uavTour1) != len(uavTour2) or uavTour1 != uavTour2
				for uavTour1, uavTour2 in zip(planResultsData['uav_tours'], executeResults['UAV_FINAL_TOURS'][i])) or \
				len(planResultsData['ugv_path']) != len(executeResults['UGV_FINAL_ORDERS'][i]) or \
				any(euclidean(planResultsData['ugv_point_map'][ugvIndex1], executeResults['UGV_FINAL_POINTS'][i][ugvIndex2]) > distTol
				for ugvIndex1, ugvIndex2 in zip(planResultsData['ugv_path'], executeResults['UGV_FINAL_ORDERS'][i])):
				numReplans += 1

		# calculate means
		numRuns = executeResults['NUM_RUNS']
		if numRuns > numFailures:
			realSuccessfulTimes.append(totalRealTime / numRuns)
		else:
			realSuccessfulTimes.append(float('NaN')) # all failures, no mean successful time
		realSuccessRates.append((numRuns - numFailures) / numRuns)
		planChangeRates.append(numReplans / numRuns)

	# plot
	plotSelfComparison(plannedTimes, realSuccessfulTimes, 'Mission Time [s]')
	plotSelfComparison(plannedSuccessRates, realSuccessRates, 'Success Rate')

	fig, ax1 = plt.subplots()
	ax1.plot(planChangeRates, [r - p for r,p in zip(realSuccessRates, plannedSuccessRates)], '.')
	ax1.set_xlabel('Ratio of Runs With Any Replan') # this is almost always = 1.0
	ax1.set_ylabel('Absolute Increase in Success Rate')
	ax2 = ax1.twinx()
	ax2.plot(planChangeRates, [100 * (p - r) / p for r,p in zip(realSuccessfulTimes, plannedTimes)], '.')
	ax2.set_ylabel('Percent Decrease in Mean Mission Time during Execution')

	fig, ax1 = plt.subplots()
	ax1.plot(
		[100 * (r - p) for r,p in zip(realSuccessRates, plannedSuccessRates)],
		[100 * (r - p) / p for r,p in zip(realSuccessfulTimes, plannedTimes)],
		'.')
	ax1.set_xlabel('Absolute Increase in Success Rate [Percent]')
	ax1.set_ylabel('Increase in Mean Mission Time during Execution [Percent]')
	ax1.grid(True)

	plt.show()
