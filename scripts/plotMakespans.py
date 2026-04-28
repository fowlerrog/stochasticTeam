
# python imports
import sys
from multiprocessing import Pool
from itertools import repeat
import numpy as np
import matplotlib.pyplot as plt

# project imports
from pathPlanning.RunnerUtils import loadYamlContents, getChildFolders, getVarFromString
from pathPlanning.Constants import planPathResultsFilename, planSettingsFilename, clusterResultsFilename
from pathPlanning.RunnerUtils import sigFigs, fillIndependentVariablesFromString
from pathPlanning.ExecuteUtils import calculatePlannedMission
from pathPlanning.PlotUtils import colorblindPalette

def getRunInfo(absResultsFolder, indVarNames, planSettings):
	d = tuple(getVarFromString(absResultsFolder, indVarName) for indVarName in indVarNames)

	# fill in plan settings data
	planSettingsData = fillIndependentVariablesFromString(planSettings, absResultsFolder)

	# load execution results from team folders
	teamFolders = [f for f in getChildFolders(absResultsFolder) if 'team_' in f]
	missionTimes = []
	for f in teamFolders:
		planResultsData = loadYamlContents(f, planPathResultsFilename, verbose=False)
		planMeanTime, planLogSuccess = calculatePlannedMission(planSettingsData, planResultsData)
		missionTimes.append(planMeanTime)

	return {
		'ind_vars' : d,
		'dep_var' : missionTimes,
	}

def getRunsInfo(parentFolder, indVarNames, planSettings):
	resultsFolders = [f for f in getChildFolders(parentFolder) if 'results_' in f]
	print(f'Loading results from {len(resultsFolders)} runs in {parentFolder}')
	# return [getRunInfo(f, indVarName) for f in resultsFolders]
	a = None
	with Pool() as p:
		a = p.starmap(getRunInfo, zip(resultsFolders, repeat(indVarNames), repeat(planSettings)))
	return a

def getMakespanInfo(absResultsFolder, indVarNames):
	d = tuple(getVarFromString(absResultsFolder, indVarName) for indVarName in indVarNames)

	clusterResults = loadYamlContents(absResultsFolder, clusterResultsFilename, verbose=False)
	pathLengths = clusterResults['tsp_lengths']

	return {
		'ind_vars' : d,
		'dep_var' : pathLengths,
	}	

def getMakespansInfo(parentFolder, indVarNames):
	resultsFolders = [f for f in getChildFolders(parentFolder) if 'results_' in f]
	print(f'Loading makespan results from {len(resultsFolders)} runs in {parentFolder}')
	# return [getRunInfo(f, indVarName) for f in resultsFolders]
	a = None
	with Pool() as p:
		a = p.starmap(getMakespanInfo, zip(resultsFolders, repeat(indVarNames)))
	return a

if __name__ == '__main__':
	# compares raw makespans with final prospect solutions

	# print help message if necessary
	if len(sys.argv) < 2 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python plotMakespans.py /path/to/results/parent/folder/')
		exit()

	folderName = sys.argv[1]

	indVarNames = ['POINTS.NUM_POINTS', 'CLUSTER.NUM_TEAMS', 'CLUSTER.HEURISTIC']

	planSettings = loadYamlContents(folderName, planSettingsFilename)
	data = getRunsInfo(folderName, indVarNames, planSettings)

	makeData = getMakespansInfo(folderName, indVarNames)

	combinedData = []
	uniqueKeys = set(d['ind_vars'] for d in data)
	for k in sorted(uniqueKeys):
		matches = [e for e in data if e['ind_vars'] == k]
		matches2 = [e for e in makeData if e['ind_vars'] == k]
		combinedData.append({'ind_vars':k, 'dep_var':[m['dep_var'] for m in matches], 'dep_var2':[m['dep_var'] for m in matches2]})

	# print results
	print(indVarNames)
	# print('\n'.join([f'{e} -> {e['dep_var']}' for e in data]))
	nsigfig = 4
	totalRealMakespans = []
	totalClusterMakespans = []
	for d in combinedData:
		realMakespans = [max(m) for m in d['dep_var']]
		clusterMakespans = [max(m) for m in d['dep_var2']]
		totalRealMakespans.extend(realMakespans)
		totalClusterMakespans.extend(clusterMakespans)
		makeRatios = [a/b for a,b in zip(realMakespans, clusterMakespans)]
		print(d['ind_vars'], 'makespan ratio =', len(makeRatios), 'values', #[sigFigs(r, nsigfig) for r in makeRatios],
		'->', sigFigs(np.mean(makeRatios), nsigfig), '+-', sigFigs(np.std(makeRatios), nsigfig))
	makeRatios = [a/b for a,b in zip(totalRealMakespans, totalClusterMakespans)]
	print('OVERALL', 'makespan ratio =', len(makeRatios), 'values', #[sigFigs(r, nsigfig) for r in makeRatios],
	   '->', sigFigs(np.mean(makeRatios), nsigfig), '+-', sigFigs(np.std(makeRatios), nsigfig))

	totalRealLengths = []
	totalClusterLengths = []
	for d in combinedData:
		realLengths = d['dep_var']
		clusterLengths = d['dep_var2']
		totalRealLengths.extend(sum(realLengths, start=[]))
		totalClusterLengths.extend(sum(clusterLengths, start=[]))
		lengthRatios = [a1/b1 for a,b in zip(realLengths, clusterLengths) for a1,b1 in zip(a,b)]
		print(d['ind_vars'], 'length ratio =', len(lengthRatios), 'values', #[sigFigs(r, nsigfig) for r in lengthRatios],
		'->', sigFigs(np.mean(lengthRatios), nsigfig), '+-', sigFigs(np.std(lengthRatios), nsigfig))
	lengthRatios = [a/b for a,b in zip(totalRealLengths, totalClusterLengths)]
	print('OVERALL', 'length ratio =', len(lengthRatios), 'values', #[sigFigs(r, nsigfig) for r in lengthRatios],
	   '->', sigFigs(np.mean(lengthRatios), nsigfig), '+-', sigFigs(np.std(lengthRatios), nsigfig))

	# plot results
	heuristicColors = colorblindPalette()
	heuristicIndex = 2

	fig, ax = plt.subplots()
	for i, h in enumerate(sorted(list(set(d[heuristicIndex] for d in uniqueKeys)))):
		thisClusterLengths = []
		thisRealLengths = []
		for e in combinedData:
			if e['ind_vars'][heuristicIndex] == h:
				thisClusterLengths.extend(sum(e['dep_var2'], start=[]))
				thisRealLengths.extend(sum(e['dep_var'], start=[]))
		color = heuristicColors[i]
		ax.plot(thisClusterLengths, thisRealLengths, '.', color=color, label=h)
	ax.set_xlabel('Planned Path Length')
	ax.set_ylabel('PRO-SPECT Path Length')
	ax.legend()
	ax.grid(True)

	fig, ax = plt.subplots()
	for i, h in enumerate(sorted(list(set(d[heuristicIndex] for d in uniqueKeys)))):
		thisClusterMakespans = []
		thisRealMakespans = []
		for e in combinedData:
			if e['ind_vars'][heuristicIndex] == h:
				thisClusterMakespans.extend(max(r) for r in e['dep_var2'])
				thisRealMakespans.extend(max(r) for r in e['dep_var'])
		color = heuristicColors[i]
		ax.plot(thisClusterMakespans, thisRealMakespans, '.', color=color, label=h)
	ax.set_xlabel('Planned Makespan')
	ax.set_ylabel('PRO-SPECT Makespan')
	ax.legend()
	ax.grid(True)

	plt.show()
