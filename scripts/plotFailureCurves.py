# python imports
import os
import re
import sys
from matplotlib import pyplot as plt
import numpy as np

# project imports
from pathPlanning.PlotUtils import plotSelfComparison
from pathPlanning.RunnerUtils import loadYamlContents, getChildFolders
from pathPlanning.Constants import executeResultsFilename, planPathResultsFilename
from pathPlanning.OurPlanner import safeExp

def getVarFromString(folderName, varName):
	regexString = varName + '_([a-zA-Z\d\.\-]+)(?:_|$)' # varName_ followed by (letters, numbers, -, .) followed by _ or end of string
	stringMatch = re.search(regexString, folderName)
	return float(stringMatch.groups(1)[0])

def getRunInfo(absResultsFolder, indVarName):
	d = getVarFromString(absResultsFolder, indVarName)

	# load planning results
	planResults = loadYamlContents(absResultsFolder, planPathResultsFilename)
	uavCycleValues = planResults['cycle_constraint_values']['UAV']
	ugvCycleValues = planResults['cycle_constraint_values']['UGV']

	# load execution results
	executeResults = loadYamlContents(absResultsFolder, executeResultsFilename)
	numCycles = len(executeResults['CYCLE_ATTEMPTS'])
	numRuns = executeResults['NUM_RUNS']
	uavFailureRates = [executeResults['UAV_TIMEOUT_FAILURES'][i] / executeResults['CYCLE_ATTEMPTS'][i] for i in range(numCycles)]
	ugvFailureRates = [executeResults['UGV_TIMEOUT_FAILURES'][i] / executeResults['CYCLE_ATTEMPTS'][i] for i in range(numCycles)]
	totalFailureRates = [len([
		executeResults['REMAINING_FLIGHT_TIMES'][j][i] for j in range(numRuns) if i < len(executeResults['REMAINING_FLIGHT_TIMES'][j]) and executeResults['REMAINING_FLIGHT_TIMES'][j][i] < 0
		]) / executeResults['CYCLE_ATTEMPTS'][i] for i in range(numCycles)]
	meanDeltas = [np.mean([
		executeResults['REMAINING_FLIGHT_TIMES'][j][i] for j in range(numRuns) if i < len(executeResults['REMAINING_FLIGHT_TIMES'][j])
	]) for i in range(numCycles)] # TODO might want to ignore negative values here

	return {
		indVarName : d,
		'uav_cycle_values' : uavCycleValues,
		'ugv_cycle_values' : ugvCycleValues,
		'uav_failure' : uavFailureRates,
		'ugv_failure' : ugvFailureRates,
		'total_failure' : totalFailureRates,
		'delta_time' : meanDeltas
	}

def getRunsInfo(parentFolder, indVarName):
	return [getRunInfo(f, indVarName) for f in getChildFolders(parentFolder)]

if __name__ == "__main__":
	# print help message if necessary
	if len(sys.argv) < 2 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python plotFailureCurves.py /path/to/results/parent/folder')
		exit()

	# TODO this folder structure is only necessary because we have no ability to do correlated plan_settings variables
	# Find child folders
	parentFolder = os.path.abspath(sys.argv[1])
	detFolder = os.path.join(parentFolder, 'deterministic')
	stochFolder = os.path.join(parentFolder, 'stochastic')
	if not os.path.isdir(detFolder) or not os.path.isdir(stochFolder):
		print('Folder %s must contain deterministic/ and stochastic/ subfolders'%parentFolder)
		exit()

	# Read result files
	detResults = getRunsInfo(detFolder, 'UAV_DELTA_TIME')
	stochResults = getRunsInfo(stochFolder, 'FAILURE_RISK')

	# Process results dicts into lists
	plannedFailureRatesStoch = []
	empiricalFailureRatesStoch = []
	empiricalDeltasStoch = []
	for run in stochResults:
		plannedFailureRatesStoch.extend(
			[1 - safeExp(logPsUav + logPsUgv) for logPsUav, logPsUgv in zip(run['uav_cycle_values'], run['ugv_cycle_values'])]
		)
		empiricalFailureRatesStoch.extend(run['total_failure'])
		empiricalDeltasStoch.extend(run['delta_time'])

	plannedDeltasDet = []
	empiricalFailureRatesDet = []
	empiricalDeltasDet = []
	for run in detResults:
		plannedDeltasDet.extend(
			[min(dtUav, dtUgv) for dtUav, dtUgv in zip(run['uav_cycle_values'], run['ugv_cycle_values'])]
		)
		empiricalFailureRatesDet.extend(run['total_failure'])
		empiricalDeltasDet.extend(run['delta_time'])

	# Plot self-comparisons
	plotSelfComparison(plannedFailureRatesStoch, empiricalFailureRatesStoch, 'Stoch', 'Cycle Failure Rate')
	plotSelfComparison(plannedDeltasDet, empiricalDeltasDet, 'Det', 'Cycle Delta Time')

	# Plot delta-prob curves
	plt.figure()
	plt.plot(empiricalDeltasDet, empiricalFailureRatesDet, '.', label='Det')
	plt.plot(empiricalDeltasStoch, empiricalFailureRatesStoch, 'x', label='Stoch')
	plt.legend()
	plt.grid(True)
	plt.xlabel('Empirical Cycle Delta [seconds]')
	plt.ylabel('Empirical Cycle Failure Rate')

	plt.show()
