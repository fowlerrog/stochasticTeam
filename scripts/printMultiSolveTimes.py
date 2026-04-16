# python imports
import sys
import numpy as np
from collections.abc import Iterable

# project imports
from pathPlanning.RunnerUtils import loadYamlContents, sigFigs
from pathPlanning.Constants import planTimeResultsFilename

def recursiveSum(l):
	if isinstance(l, Iterable):
		return sum(recursiveSum(e) for e in l)
	return l

if __name__ == '__main__':
	# print help message if necessary
	if len(sys.argv) < 2 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python printMultiSolveTimes.py /path/to/%s'%(planTimeResultsFilename))
		exit()

	planTimeResultsFile = sys.argv[1]
	yamlContents = loadYamlContents(planTimeResultsFile, planTimeResultsFilename)

	# get independent variable names
	xKey = ['POINTS.NUM_POINTS', 'CLUSTER.NUM_TEAMS', 'CLUSTER.HEURISTIC']

	# get dependent variable names
	yKey = ['CLUSTER_TIME', 'TOTAL_TIME']

	# convert yaml to dict list
	numRuns = len(yamlContents[xKey[0]])
	data = [ {
			'ind_vars' : tuple(yamlContents[x][n] for x in xKey),
			'dep_var' : sum(recursiveSum(yamlContents[y][n]) for y in yKey)
		} for n in range(numRuns)
	]

	# combine groups and print
	combinedData = []
	uniqueKeys = set(d['ind_vars'] for d in data)
	for k in sorted(uniqueKeys):
		matches = [e for e in data if e['ind_vars'] == k]
		combinedData.append({'ind_vars':k, 'dep_var':[m['dep_var'] for m in matches]})

	print(xKey)
	# print('\n'.join([f'{e} -> {e['dep_var']}' for e in data]))
	nsigfig = 4
	for d in combinedData:
		rates = d['dep_var']
		print(d['ind_vars'], '=', [sigFigs(r, nsigfig) for r in rates], '->', sigFigs(np.mean(rates), nsigfig), '+-', sigFigs(np.std(rates), nsigfig))
