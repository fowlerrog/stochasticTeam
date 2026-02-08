# python imports
import sys
import traceback
from multiprocessing import Pool
from itertools import repeat

# project imports
from pathPlanning.Constants import executeSettingsFilename, planSettingsFilename, planPathResultsFilename
from pathPlanning.ExecuteUtils import executePlanFromSettings

def tryToExecutePlan(exSettings, planSettings, planResults):
	try:
		return executePlanFromSettings(exSettings, planSettings, planResults)
	except Exception:
		print(traceback.format_exc())

if __name__ == '__main__':
	# print help message if necessary
	if len(sys.argv) < 4 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python executePlan.py /path/to/%s /path/to/%s /path/to/%s [/path/to/another/%s ...]'%(executeSettingsFilename, planSettingsFilename, planPathResultsFilename, planPathResultsFilename))
		exit()
	
	# for each provided settings file, execute plan (in parallel)
	with Pool() as p:
		_ = p.starmap(tryToExecutePlan, zip(repeat(sys.argv[1]), repeat(sys.argv[2]), sys.argv[3:]))
