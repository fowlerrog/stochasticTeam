# python imports
import sys
import numpy as np
from copy import deepcopy

# project imports
from pathPlanning.Constants import planTimeResultsFilename
from pathPlanning.RunnerUtils import loadYamlContents, getVarFromString, sigFigs

if __name__ == '__main__':
	# print help message if necessary
	# more specifically, this script is for comparing refined and unrefined computation times (optional step 3 of the algorithm)
	if len(sys.argv) < 2 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python printMissionTimeComparison.py /path/to/%s'%(planTimeResultsFilename))
		exit()

	# load yaml
	results = loadYamlContents(sys.argv[1])
	nRuns = len(results[next(iter(results))]) # length of value of first key

	# group data together by common independent variables
	indVarNames = ['PLANNER.TYPE', 'POINTS.NUM_POINTS', 'SEED', 'PLANNER.FAILURE_RISK', 'PLANNER.TYPE']
	refineVarName = 'PLANNER.REFINE_TOURS'
	depVarNames = ['TOTAL_TIME', 'REFINE_TIME']
	allVars = indVarNames + [refineVarName] + depVarNames

	pairs = []
	i = 0
	resultsList = [{k:results[k][i] for k in allVars} for i in range(nRuns)] # convert dict of lists -> list of dicts
	while i < len(resultsList) - 1:
		j = i + 1
		found = False
		while j < len(resultsList):
			if all(resultsList[i][k] == resultsList[j][k] for k in indVarNames):
				print('Match found for', {k:resultsList[i][k] for k in indVarNames})
				if resultsList[i][refineVarName] == True: # this is the refined one
					pairs.append([resultsList[i], resultsList[j]])
				else:
					pairs.append([resultsList[j], resultsList[i]])
				resultsList.pop(max(i,j))
				resultsList.pop(min(i,j))
				found = True
				break
			j += 1
		if not found:
			print('NO MATCH FOUND FOR', {k:resultsList[i][k] for k in indVarNames})
			i += 1
	if len(pairs) == 0:
		print('No refined/unrefined pairs found')
		exit()

	# evaluate ratios
	nSigFig = 4
	numPointsVar = 'POINTS.NUM_POINTS'
	sortedNumPoints = sorted(list(set(results[numPointsVar])))
	print('ratios are: refined/unrefined reported as (mean,sigma) and [min,max]')
	print('n\t' + '\t'.join(depVarNames))
	for n in sortedNumPoints:
		thisPairs = [p for p in pairs if p[0][numPointsVar] == n]
		line1 = f'{n}' # means and stds line
		line2 = '' # mins and maxs line
		for v in depVarNames:
			ratios = [p[0][v] / p[1][v] for p in thisPairs]
			line1 += f'\t({sigFigs(np.nanmean(ratios), nSigFig)}, {sigFigs(np.nanstd(ratios), nSigFig)})'
			line2 += f'\t[{sigFigs(np.nanmin(ratios), nSigFig)}, {sigFigs(np.nanmax(ratios), nSigFig)}]'
		print(line1 + '\n' + line2)
