# python imports
import os
import re
import sys
from matplotlib import pyplot as plt

# project imports
from pathPlanning.RunnerUtils import loadYamlContents, getChildFolders
from pathPlanning.Constants import executeResultsFilename

def getFailureRate(executeResults):
	# TODO total failure rate is the product of several independent cycles, each with their own failure rate < p_r
	return (sum(executeResults['UAV_TIMEOUT_FAILURES']) + sum(executeResults['UGV_TIMEOUT_FAILURES'])) / sum(executeResults['CYCLE_ATTEMPTS'])

def getDelta(executeResults):
	n = sum( len(x) for x in executeResults['REMAINING_FLIGHT_TIMES'] )
	s = sum( sum(x) for x in executeResults['REMAINING_FLIGHT_TIMES'] )
	return s / n

def getVarFromString(folderName, varName):
	regexString = varName + '_(\d+\.?\d*)'
	stringMatch = re.search(regexString, folderName)
	return float(stringMatch.groups(1)[0])

def getRunInfo(absResultsFolder, indVarName):
	d = getVarFromString(absResultsFolder, indVarName)
	executeResults = loadYamlContents(absResultsFolder, executeResultsFilename)
	return [d, getFailureRate(executeResults), getDelta(executeResults)]

def getRunsInfo(parentFolder, indVarName):
	return [getRunInfo(f, indVarName) for f in getChildFolders(parentFolder)]

def plotSelfComparison(resultsList, index1, index2, labelString='', varName=''):
	xData = [a[index1] for a in resultsList]
	yData = [a[index2] for a in resultsList]
	propLineMin = max(min(xData), min(yData))
	propLineMax = min(max(xData), max(yData))
	plt.figure()
	plt.plot(xData, yData, '.k', label=labelString)
	plt.plot([propLineMin, propLineMax], [propLineMin, propLineMax], ':k')
	plt.legend()
	plt.grid(True)
	plt.xlabel('Planned ' + varName)
	plt.ylabel('Empirical ' + varName)

if __name__ == "__main__":
	# print help message if necessary
	if len(sys.argv) < 2 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python plotFailureCurves.py /path/to/results/parent/folder')
		exit()

	parentFolder = os.path.abspath(sys.argv[1])
	detFolder = os.path.join(parentFolder, 'deterministic')
	stochFolder = os.path.join(parentFolder, 'stochastic')
	if not os.path.isdir(detFolder) or not os.path.isdir(stochFolder):
		print('Folder %s must contain deterministic/ and stochastic/ subfolders'%parentFolder)
		exit()

	detResults = getRunsInfo(detFolder, 'UAV_DELTA_TIME')
	stochResults = getRunsInfo(stochFolder, 'FAILURE_RISK')

	plt.figure()
	# plt.plot([a[0] for a in detResults], [a[1] for a in detResults], '.', label='Det')
	# plt.plot([a[2] for a in stochResults], [a[0] for a in stochResults], '.', label='Stoch')
	plt.plot([a[2] for a in detResults], [a[1] for a in detResults], '.', label='Det')
	plt.plot([a[2] for a in stochResults], [a[1] for a in stochResults], 'x', label='Stoch')
	plt.legend()
	plt.grid(True)
	plt.xlabel('Empirical Delta [seconds]')
	plt.ylabel('Empirical Failure Rate')

	plotSelfComparison(stochResults, 0, 1, 'Stoch', 'Failure Rate')
	plotSelfComparison(detResults, 0, 2, 'Det', 'Delta Time')

	plt.show()
