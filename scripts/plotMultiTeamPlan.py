# python imports
import sys
import traceback

# project imports
from pathPlanning.PlotUtils import plotMultiTeamPlanFromPlanResults

if __name__ == '__main__':
	# print help message if necessary
	if len(sys.argv) < 2 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python plotMultiTeamPlan.py [-s] [-f] /path/to/run/results/folder/ [/path/to/run/results/folder2/ ...]')
		exit()

	i = 1
	simple = False
	full = False
	while sys.argv[i][0] == '-':
		if sys.argv[i] == '-s':
			simple = True
		elif sys.argv[i] == '-f':
			full = True
		i += 1

	# for all provided settings files, run plotter
	try:
		plotMultiTeamPlanFromPlanResults(sys.argv[i:], full=full, simple=simple)
	except Exception:
		print(traceback.format_exc())
