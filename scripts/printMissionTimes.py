# python imports
import sys
import os
import numpy as np

# project imports
from pathPlanning.ExecuteUtils import calculatePlannedMission
from pathPlanning.RunnerUtils import loadYamlContents, fillIndependentVariablesFromString, toDir
from pathPlanning.Constants import planSettingsFilename, planPathResultsFilename

if __name__ == '__main__':
	# print help message if necessary
	if len(sys.argv) < 2 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python printMissionTimes.py /path/to/results/folder/ [/path/to/results/folder2/ ...]')
		exit()

	folderNames = [toDir(s) for s in sys.argv[1:]]

	plannedTimes = [] # total planned mean times
	numPoints = [] # number of points in each plan

	for f in folderNames:
		# look for path settings one folder above (this is brittle)
		planFolder = os.path.split(f)[0]
		planSettingsData = loadYamlContents(planFolder, planSettingsFilename, verbose=False)

		# choose independent plan variables
		thisPlanSettingsData = fillIndependentVariablesFromString(planSettingsData, f)

		# load plan results
		planResultsData = loadYamlContents(f, planPathResultsFilename, verbose=False)
		plannedTime, plannedLogSuccessRate = calculatePlannedMission(thisPlanSettingsData, planResultsData)
		plannedTimes.append(plannedTime)
		numPoints.append(len(planResultsData['uav_points']))

	# combine groups and print
	nUnique = [numPoints[i] for i in range(len(numPoints)) if numPoints[i] not in numPoints[:i]]
	print('Statistics:')
	for n in sorted(nUnique):
		thisNTimes = [plannedTimes[i] for i in range(len(numPoints)) if numPoints[i] == n]
		minTime = np.min(thisNTimes)
		meanTime = np.mean(thisNTimes)
		maxTime = np.max(thisNTimes)
		stdDev = np.std(thisNTimes)
		print(f"n={n}: min {minTime:.2f} mean {meanTime:.2f} max {maxTime:.2f} -> mu sigma {meanTime:.2f} +- {stdDev:.2f}")
