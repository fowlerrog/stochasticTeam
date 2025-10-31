# python imports
import sys
import traceback

# project imports
sys.path.append('../src')
from pathPlanning.Constants import executeSettingsFilename, planSettingsFilename, planPathResultsFilename
from pathPlanning.ExecuteUtils import executePlanFromSettings

if __name__ == '__main__':
	# print help message if necessary
	if len(sys.argv) < 4 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python executePlan.py /path/to/%s /path/to/%s /path/to/%s [/path/to/another/%s ...]'%(executeSettingsFilename, planSettingsFilename, planPathResultsFilename, planPathResultsFilename))
		exit()
	
	# for each provided settings file, run planner
	for s in sys.argv[3:]:
		try:
			executePlanFromSettings(sys.argv[1], sys.argv[2], s)
		except Exception:
			print(traceback.format_exc())
