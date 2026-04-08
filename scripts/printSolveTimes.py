# python imports
import sys
import os
import numpy as np

# project imports
from pathPlanning.ExecuteUtils import calculatePlannedMission
from pathPlanning.RunnerUtils import loadYamlContents, fillIndependentVariablesFromString, toDir, getVarFromString
from pathPlanning.Constants import planTimeResultsFilename

if __name__ == '__main__':
	# print help message if necessary
	if len(sys.argv) < 2 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python printSolveTimes.py /path/to/%s'%(planTimeResultsFilename))
		exit()

	planTimeResultsFile = sys.argv[1]
	yamlContents = loadYamlContents(planTimeResultsFile)

	# get independent variable name
	xKey = 'POINTS.NUM_POINTS'

	# get dependent variable name
	yKey = 'TOTAL_TIME'

	plannedTimes = [] # total planned mean times
	numPoints = [] # number of points in each plan
	seeds = [] # random seeds
	compTimes = [] # computation times

	for f in folderNames:
		# look for path settings one folder above (this is brittle)
		planFolder = os.path.split(f)[0]
		planSettingsData = loadYamlContents(planFolder, planSettingsFilename)

		# choose independent plan variables
		thisPlanSettingsData = fillIndependentVariablesFromString(planSettingsData, f)
		seed = int(getVarFromString(f, 'SEED'))
		n = int(getVarFromString(f, xKey))

		# load plan results
		planResultsData = loadYamlContents(f, planPathResultsFilename)
		plannedTime, plannedLogSuccessRate = calculatePlannedMission(thisPlanSettingsData, planResultsData)
		plannedTimes.append(plannedTime)
		numPoints.append(len(planResultsData['uav_points']))
		seeds.append(seed)

		# get comp time
		yamlIndex = [i for i in range(len(yamlContents[xKey])) if yamlContents['SEED'][i] == seed and yamlContents[xKey][i] == n]
		assert len(yamlIndex) == 1, 'duplicate or missing seed/n combo'
		compTimes.append(yamlContents[yKey][yamlIndex[0]])

	# combine groups and print
	nUnique = [numPoints[i] for i in range(len(numPoints)) if numPoints[i] not in numPoints[:i]]
	with open(outputFile, 'w') as f:
		f.write('N, seed, missionTime, compTime\n')
		for i in range(len(plannedTimes)):
			f.write(f'{numPoints[i]}, {seeds[i]}, {plannedTimes[i]:.2f}, {compTimes[i]:.4f}\n')

		f.write('\nMission Time Statistics:\n')
		for n in sorted(nUnique):
			thisNTimes = [plannedTimes[i] for i in range(len(numPoints)) if numPoints[i] == n]
			minTime = np.min(thisNTimes)
			meanTime = np.mean(thisNTimes)
			maxTime = np.max(thisNTimes)
			stdDev = np.std(thisNTimes)
			f.write(f"n={n}: min {minTime:.2f} mean {meanTime:.2f} max {maxTime:.2f} -> mu sigma {meanTime:.2f} +- {stdDev:.2f}\n")

		f.write('\nComp Time Statistics:\n')
		for n in sorted(nUnique):
			thisNTimes = [compTimes[i] for i in range(len(numPoints)) if numPoints[i] == n]
			minTime = np.min(thisNTimes)
			meanTime = np.mean(thisNTimes)
			maxTime = np.max(thisNTimes)
			stdDev = np.std(thisNTimes)
			f.write(f"n={n}: min {minTime:.2f} mean {meanTime:.2f} max {maxTime:.2f} -> mu sigma {meanTime:.4f} +- {stdDev:.4f}\n")