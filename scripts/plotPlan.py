# python imports
import sys
import traceback

# project imports
from pathPlanning.PlotUtils import plotPlanFromFolder

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
