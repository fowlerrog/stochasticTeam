
# python imports
import os
import sys
import traceback
import yaml
import matplotlib.pyplot as plt

# project imports
from PlotUtils import *
from Constants import planPathResultsFilename
from RunnerUtils import loadYamlContents

def loadPlanResultsFromFolder(folderPath):
	"""Loads planning results from a yaml in a result folder"""

	absFile = os.path.abspath(os.path.join(folderPath, planPathResultsFilename))
	print('Loading planning results from', absFile)
    
	# load run parameters from yaml
	params = loadYamlContents(folderPath, planPathResultsFilename)
	if len(params) == 0:
		print('Params not found')
		return

	# standardize:
	# ugv_mapping_to_points keys should be ints TODO is this necessary with yaml?
	params['ugv_mapping_to_points'] = {int(k):v for k,v in params['ugv_mapping_to_points'].items()}
	
	return params

def plotPlanFromFolder(folderPath):
	"""Generates plots for a TSP path from a results folder"""
	absFolderPath = os.path.abspath(folderPath)

	plan = loadPlanResultsFromFolder(absFolderPath)
	if plan is None:
		return

	TSP_figure_name = os.path.join(absFolderPath, 'TSP_path.png')
	if 'uav_points' in plan:
		plot_path(plan['uav_points'], filename=TSP_figure_name, show=False)
	# else:
	# 	# If no solution, just scatter the reordered points for reference
	# 	plot_points(points,
	# 				filename=TSP_figure_name)

	cycles_figure_name = os.path.join(absFolderPath, 'uav_cycles.png')
	plot_cycles(plan['uav_cycles'],
			    plan['uav_points'],
				plan["ugv_mapping_to_points"],
				plan["ugv_path"],
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
