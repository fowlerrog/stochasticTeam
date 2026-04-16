# python imports
import sys
from multiprocessing import Pool
from itertools import repeat
import numpy as np

# project imports
from pathPlanning.RunnerUtils import loadYamlContents, toDir, getChildFolders, getVarFromString
from pathPlanning.Constants import executeResultsFilename
from pathPlanning.RunnerUtils import sigFigs

def getRunInfo(absResultsFolder, indVarNames):
	d = tuple(getVarFromString(absResultsFolder, indVarName) for indVarName in indVarNames)

	# load execution results from team folders
	teamFolders = [f for f in getChildFolders(absResultsFolder) if 'team_' in f]
	maxFailureRate = float('nan')
	for f in teamFolders:
		executeResults = loadYamlContents(f, executeResultsFilename, verbose=False)
		numRuns = executeResults['NUM_RUNS']
		totalFailureRate = len([attempt for attempt in executeResults['REMAINING_FLIGHT_TIMES'] if any(a < 0 for a in attempt)]) / numRuns
		maxFailureRate = max(totalFailureRate, maxFailureRate)

	return {
		'ind_vars' : d,
		'total_failure' : maxFailureRate,
	}

def getRunsInfo(parentFolder, indVarNames):
	resultsFolders = [f for f in getChildFolders(parentFolder) if 'results_' in f]
	print(f'Loading results from {len(resultsFolders)} runs in {parentFolder}')
	# return [getRunInfo(f, indVarName) for f in resultsFolders]
	with Pool() as p:
		return p.starmap(getRunInfo, zip(resultsFolders, repeat(indVarNames)))

if __name__ == '__main__':
	# print help message if necessary
	if len(sys.argv) < 2 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python printMultiFailureRates.py /path/to/results/parent/folder/')
		exit()

	folderName = toDir(sys.argv[1])

	# independent variables (TODO this should not be a manual change)
	# indVarNames = ['PLANNER.FAILURE_RISK', 'POINTS.NUM_POINTS']
	# indVarNames = ['POINTS.NUM_POINTS']
	indVarNames = ['POINTS.NUM_POINTS', 'CLUSTER.NUM_TEAMS', 'CLUSTER.HEURISTIC']

	# get info
	data = getRunsInfo(folderName, indVarNames)

	combinedData = []
	uniqueKeys = set(d['ind_vars'] for d in data)
	for k in sorted(uniqueKeys):
		matches = [e for e in data if e['ind_vars'] == k]
		combinedData.append({'ind_vars':k, 'total_failure':[m['total_failure'] for m in matches]})

	print(indVarNames)
	# print('\n'.join([f'{e} -> {e['total_failure']}' for e in data]))
	nsigfig = 4
	for d in combinedData:
		rates = d['total_failure']
		print(d['ind_vars'], '=', rates, '->', sigFigs(np.mean(rates), nsigfig), '+-', sigFigs(np.std(rates), nsigfig))
