# python imports
import sys
import traceback
import os

# project imports
from pathPlanning.Constants import executeSettingsFilename, planSettingsFilename
from pathPlanning.ExecuteUtils import executePlanFromSettings
from pathPlanning.PlannerUtils import runPlannerFromSettings
from pathPlanning.RunnerUtils import toDir, getChildFolders

if __name__ == '__main__':
	# print help message if necessary
	if len(sys.argv) < 3 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python runFull.py /path/to/%s /path/to/%s'%(planSettingsFilename, executeSettingsFilename))
		exit()

	pathSettingsFile = sys.argv[1]
	executeSettingsFile = sys.argv[2]

	# run the planner
	try:
		runPlannerFromSettings(pathSettingsFile)
	except Exception:
		print(traceback.format_exc())
		exit()

	# find all results folders
	print(pathSettingsFile)
	planSettingsFolder = toDir(pathSettingsFile)
	print(planSettingsFolder)
	resultsFolders = [f for f in getChildFolders(planSettingsFolder) if 'results_' in os.path.split(f)[1]]
	print(resultsFolders)

	# execute on each results folder
	for f in resultsFolders:
		try:
			executePlanFromSettings(executeSettingsFile, pathSettingsFile, f)
		except Exception:
			print(traceback.format_exc())
