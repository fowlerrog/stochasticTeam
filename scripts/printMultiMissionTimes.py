# python imports
import sys
from multiprocessing import Pool
from itertools import repeat
import numpy as np

# project imports
from pathPlanning.RunnerUtils import loadYamlContents, toDir, getChildFolders, getVarFromString
from pathPlanning.Constants import planPathResultsFilename, planSettingsFilename
from pathPlanning.RunnerUtils import sigFigs, fillIndependentVariablesFromString
from pathPlanning.ExecuteUtils import calculatePlannedMission

def getRunInfo(absResultsFolder, indVarNames, planSettings):
	d = tuple(getVarFromString(absResultsFolder, indVarName) for indVarName in indVarNames)

	# fill in plan settings data
	planSettingsData = fillIndependentVariablesFromString(planSettings, absResultsFolder)

	# load execution results from team folders
	teamFolders = [f for f in getChildFolders(absResultsFolder) if 'team_' in f]
	maxMissionTime = float('nan')
	for f in teamFolders:
		planResultsData = loadYamlContents(f, planPathResultsFilename, verbose=False)
		planMeanTime, planLogSuccess = calculatePlannedMission(planSettingsData, planResultsData)
		maxMissionTime = max(planMeanTime, maxMissionTime)

	return {
		'ind_vars' : d,
		'dep_var' : maxMissionTime,
	}

def getRunsInfo(parentFolder, indVarNames, planSettings):
	resultsFolders = [f for f in getChildFolders(parentFolder) if 'results_' in f]
	print(f'Loading results from {len(resultsFolders)} runs in {parentFolder}')
	# return [getRunInfo(f, indVarName) for f in resultsFolders]
	with Pool() as p:
		return p.starmap(getRunInfo, zip(resultsFolders, repeat(indVarNames), repeat(planSettings)))

if __name__ == '__main__':
	# print help message if necessary
	if len(sys.argv) < 2 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python printMultiMissionTimes.py /path/to/results/parent/folder/')
		exit()

	folderName = toDir(sys.argv[1])

	# independent variables (TODO this should not be a manual change)
	# indVarNames = ['PLANNER.FAILURE_RISK', 'POINTS.NUM_POINTS']
	# indVarNames = ['POINTS.NUM_POINTS']
	indVarNames = ['POINTS.NUM_POINTS', 'CLUSTER.NUM_TEAMS', 'CLUSTER.HEURISTIC']

	# load plan settings
	planSettings = loadYamlContents(folderName, planSettingsFilename)

	# get info
	data = getRunsInfo(folderName, indVarNames, planSettings)

	combinedData = []
	uniqueKeys = set(d['ind_vars'] for d in data)
	for k in sorted(uniqueKeys):
		matches = [e for e in data if e['ind_vars'] == k]
		combinedData.append({'ind_vars':k, 'dep_var':[m['dep_var'] for m in matches]})

	print(indVarNames)
	# print('\n'.join([f'{e} -> {e['dep_var']}' for e in data]))
	nsigfig = 4
	for d in combinedData:
		rates = d['dep_var']
		print(d['ind_vars'], '=', [sigFigs(r, nsigfig) for r in rates], '->', sigFigs(np.mean(rates), nsigfig), '+-', sigFigs(np.std(rates), nsigfig))
