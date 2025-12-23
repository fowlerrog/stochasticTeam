# python imports
import os
import sys
from matplotlib import pyplot as plt
import numpy as np
from math import prod
from multiprocessing import Pool
from itertools import repeat

# project imports
from pathPlanning.PlotUtils import plotSelfComparison
from pathPlanning.RunnerUtils import loadYamlContents, getChildFolders, getVarFromString
from pathPlanning.Constants import executeResultsFilename, planPathResultsFilename
from pathPlanning.OurPlanner import safeExp

def getRunInfo(absResultsFolder, indVarName):
	d = float(getVarFromString(absResultsFolder, indVarName))

	# load planning results
	planResults = loadYamlContents(absResultsFolder, planPathResultsFilename, verbose=False)
	uavTourValues = planResults['tour_constraint_values']['UAV']
	ugvTourValues = planResults['tour_constraint_values']['UGV']

	# load execution results
	executeResults = loadYamlContents(absResultsFolder, executeResultsFilename, verbose=False)
	numTours = len(executeResults['TOUR_ATTEMPTS'])
	numRuns = executeResults['NUM_RUNS']
	uavFailureRates = [executeResults['UAV_TIMEOUT_FAILURES'][i] / executeResults['TOUR_ATTEMPTS'][i] for i in range(numTours)]
	ugvFailureRates = [executeResults['UGV_TIMEOUT_FAILURES'][i] / executeResults['TOUR_ATTEMPTS'][i] for i in range(numTours)]
	totalFailureRates = [len([
		executeResults['REMAINING_FLIGHT_TIMES'][j][i] for j in range(numRuns) if i < len(executeResults['REMAINING_FLIGHT_TIMES'][j]) and executeResults['REMAINING_FLIGHT_TIMES'][j][i] < 0
		]) / executeResults['TOUR_ATTEMPTS'][i] for i in range(numTours)]
	meanDeltas = [np.mean([
		executeResults['REMAINING_FLIGHT_TIMES'][j][i] for j in range(numRuns) if i < len(executeResults['REMAINING_FLIGHT_TIMES'][j])
	]) for i in range(numTours)] # TODO might want to ignore negative values here

	return {
		indVarName : d,
		'uav_tour_values' : uavTourValues,
		'ugv_tour_values' : ugvTourValues,
		'uav_failure' : uavFailureRates,
		'ugv_failure' : ugvFailureRates,
		'total_failure' : totalFailureRates,
		'delta_time' : meanDeltas
	}

def getRunsInfo(parentFolder, indVarName):
	resultsFolders = [f for f in getChildFolders(parentFolder) if 'results_' in f]
	print(f'Loading results from {len(resultsFolders)} runs in {parentFolder}')
	# return [getRunInfo(f, indVarName) for f in resultsFolders]
	p = Pool()
	return p.starmap(getRunInfo, zip(resultsFolders, repeat(indVarName)))

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
	print('Done loading')

	# Process results dicts into lists
	plannedTourFailureRatesStoch = []
	empiricalTourFailureRatesStoch = []
	empiricalTourDeltasStoch = []
	plannedPlanFailureRatesStoch = []
	empiricalPlanFailureRatesStoch = []
	empiricalPlanDeltasStoch = []
	for run in stochResults:
		plannedTourFailureRatesStoch.extend(
			[1 - safeExp(logPsUav + logPsUgv) for logPsUav, logPsUgv in zip(run['uav_tour_values'], run['ugv_tour_values'])]
		)
		empiricalTourFailureRatesStoch.extend(run['total_failure'])
		empiricalTourDeltasStoch.extend(run['delta_time'])
		plannedPlanFailureRatesStoch.append(run['FAILURE_RISK'])
		empiricalPlanFailureRatesStoch.append(1 - prod([1 - p for p in run['total_failure']]))
		empiricalPlanDeltasStoch.append(np.mean(run['delta_time']))

	plannedTourDeltasDet = []
	empiricalTourFailureRatesDet = []
	empiricalTourDeltasDet = []
	plannedPlanDeltasDet = []
	empiricalPlanFailureRatesDet = []
	empiricalPlanDeltasDet = []
	for run in detResults:
		plannedTourDeltasDet.extend(
			[min(dtUav, dtUgv) for dtUav, dtUgv in zip(run['uav_tour_values'], run['ugv_tour_values'])]
		)
		empiricalTourFailureRatesDet.extend(run['total_failure'])
		empiricalTourDeltasDet.extend(run['delta_time'])
		plannedPlanDeltasDet.append(run['UAV_DELTA_TIME'])
		empiricalPlanFailureRatesDet.append(1 - prod([1 - p for p in run['total_failure']]))
		empiricalPlanDeltasDet.append(np.mean(run['delta_time']))

	# Plot self-comparisons
	plotSelfComparison(plannedTourFailureRatesStoch, empiricalTourFailureRatesStoch, 'Stoch', 'Tour Failure Rate')
	plotSelfComparison(plannedPlanFailureRatesStoch, empiricalPlanFailureRatesStoch, 'Stoch', 'Plan Failure Rate')
	plotSelfComparison(plannedTourDeltasDet, empiricalTourDeltasDet, 'Det', 'Tour Delta Time')
	plotSelfComparison(plannedPlanDeltasDet, empiricalPlanDeltasDet, 'Det', 'Plan Mean Delta Time')

	# Plot delta-prob curves (by tour)
	plt.figure()
	plt.plot(empiricalTourDeltasDet, empiricalTourFailureRatesDet, '.', label='Det')
	plt.plot(empiricalTourDeltasStoch, empiricalTourFailureRatesStoch, 'x', label='Stoch')
	plt.legend()
	plt.grid(True)
	plt.xlabel('Empirical Tour Delta [seconds]')
	plt.ylabel('Empirical Tour Failure Rate')

	# Plot delta-prob curves (by plan)
	plt.figure()
	plt.plot(empiricalPlanDeltasDet, empiricalPlanFailureRatesDet, '.', label='Det')
	plt.plot(empiricalPlanDeltasStoch, empiricalPlanFailureRatesStoch, 'x', label='Stoch')
	plt.legend()
	plt.grid(True)
	plt.xlabel('Empirical Plan Mean Delta [seconds]')
	plt.ylabel('Empirical Plan Failure Rate')

	# TODO compare with actual found-plan values, not just user-specified vs. empirical
	# TODO collect and plot execution times

	plt.show()
