# python imports
import sys
import os
import matplotlib.pyplot as plt
import traceback

# project imports
from pathPlanning.Constants import planTimeResultsFilename
from pathPlanning.RunnerUtils import loadYamlContents, toDir
from pathPlanning.PlotUtils import plotSolveTimes

if __name__ == '__main__':
	# print help message if necessary
	if len(sys.argv) < 2 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python plotSolveTimes.py /path/to/%s [/path/to/another/%s ...]'%(planTimeResultsFilename, planTimeResultsFilename))
		exit()

	# get independent variable name
	indVarName = 'NUM_POINTS'

	# for each plan time results, plot curve
	fig = None
	ax = None
	for s in sys.argv[1:]:
		try:
			endFolderName = os.path.split(toDir(s))[1]
			referenceExp = None
			depVarName = None
			if 'deterministic' in endFolderName:
				referenceExp = 3
				depVarName = ['COST_TIME', 'CYCLE_TIME', 'TSP_TIME', 'UGV_TIME']
			elif 'stochastic' in endFolderName:
				referenceExp = 5
				depVarName = 'TOTAL_TIME'
			else:
				print('Folder type not recognized')
				continue

			yamlContents = loadYamlContents(s, planTimeResultsFilename)
			fig, ax = plotSolveTimes(yamlContents, 'NUM_POINTS', 'TOTAL_TIME',
				fig=fig, ax=ax,
				labelString=endFolderName,
				referenceExp=referenceExp
			)
		except Exception:
			print('Failure during plotting of', s)
			print(traceback.format_exc())

	plt.grid(True)
	plt.xlabel(indVarName)
	plt.ylabel(depVarName)
	plt.legend()
	plt.show()
