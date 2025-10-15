
# python imports
import os
import sys
import traceback
import json
import matplotlib.pyplot as plt

# project imports
from PlotUtils import *

def loadPlanResultsFromFolder(folderPath):
	"""Loads planning results from a json in a result folder"""

	defaultFileName = 'plan_results.json'
	absFile = os.path.abspath(os.path.join(folderPath, defaultFileName))
	print('Loading planning results from', absFile)
    
	# load run parameters from json
	params = {}
	with open(absFile, 'r') as f:
		try:
			params = json.load(f)
		except Exception:
			print(traceback.format_exc())
	if len(params) == 0:
		print('Params not found')
		return

	# standardize:
	# mapping_to_points keys should be ints
	params['mapping_to_points'] = {int(k):v for k,v in params['mapping_to_points'].items()}
	
	return params

def plotPlanFromFolder(folderPath):
	"""Generates plots for a TSP path from a results folder"""
	absFolderPath = os.path.abspath(folderPath)

	plan = loadPlanResultsFromFolder(absFolderPath)
	if plan is None:
		return

	TSP_figure_name = os.path.join(absFolderPath, 'TSP_path.png')
	if 'tsp_points' in plan:
		plot_path(plan['tsp_points'], filename=TSP_figure_name, show=False)
	# else:
	# 	# If no solution, just scatter the reordered points for reference
	# 	plot_points(points,
	# 				filename=TSP_figure_name)

	cycles_figure_name = os.path.join(absFolderPath, 'clusters.png')
	plot_clusters(plan['cycles'],
			    plan["clusters"],
				plan['tsp_points'],
				plan["mapping_to_points"],
				plan["path"],
				cycles_figure_name,
				False)
	
	plt.show()

### main function
if __name__ == '__main__':
	# print help message if necessary
	if len(sys.argv) < 2 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python plotPlan.py /path/to/run/results/folder/ [/path/to/run/results/folder2/ ...]')
		exit()
	
	# for each provided settings file, run plotter
	for s in sys.argv[1:]:
		try:
			plotPlanFromFolder(s)
		except Exception:
			print(traceback.format_exc())
