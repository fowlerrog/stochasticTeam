# python imports
import sys
from multiprocessing import Pool
from itertools import repeat
import numpy as np

# project imports
from pathPlanning.RunnerUtils import loadYamlContents, toDir, getChildFolders, getVarFromString
from pathPlanning.Constants import executeResultsFilename

def getRunInfo(absResultsFolder, indVarNames):
	d = tuple(float(getVarFromString(absResultsFolder, indVarName)) for indVarName in indVarNames)

	# load execution results
	executeResults = loadYamlContents(absResultsFolder, executeResultsFilename, verbose=False)
	numRuns = executeResults['NUM_RUNS']
	totalFailureRate = len([attempt for attempt in executeResults['REMAINING_FLIGHT_TIMES'] if any(a < 0 for a in attempt)]) / numRuns

	return {
		'ind_vars' : d,
		'total_failure' : totalFailureRate,
	}

def getRunsInfo(parentFolder, indVarNames):
	resultsFolders = [f for f in getChildFolders(parentFolder) if 'results_' in f]
	print(f'Loading results from {len(resultsFolders)} runs in {parentFolder}')
	# return [getRunInfo(f, indVarName) for f in resultsFolders]
	with Pool() as p:
		return p.starmap(getRunInfo, zip(resultsFolders, repeat(indVarNames)))

def sigfigs(i, n):
	return '{:g}'.format(float('{:.{p}g}'.format(i, p=n)))

if __name__ == '__main__':
	# print help message if necessary
	if len(sys.argv) < 2 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python printFailureRates.py /path/to/results/parent/folder/')
		exit()

	folderName = toDir(sys.argv[1])

	# independent variables
	indVarNames = ['PLANNER.FAILURE_RISK', 'POINTS.NUM_POINTS']
	# indVarNames = ['POINTS.NUM_POINTS']

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
		print(d['ind_vars'], '=', rates, '->', sigfigs(np.mean(rates), nsigfig), '+-', sigfigs(np.std(rates), nsigfig))
