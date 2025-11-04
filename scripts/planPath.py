# python imports
import sys
import traceback

# project imports
from pathPlanning.Constants import planSettingsFilename
from pathPlanning.PlannerUtils import runPlannerFromSettings

if __name__ == '__main__':
	# print help message if necessary
	if len(sys.argv) < 2 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python runPlanner.py /path/to/%s [/path/to/another/%s ...]'%tuple(2*[planSettingsFilename]))
		exit()
	
	# for each provided settings file, run planner
	for s in sys.argv[1:]:
		try:
			runPlannerFromSettings(s)
		except Exception:
			print(traceback.format_exc())
