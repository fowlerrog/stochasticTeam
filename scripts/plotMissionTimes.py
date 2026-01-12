# python imports
import sys
import os
import matplotlib.pyplot as plt
import traceback
import numpy as np
from multiprocessing import Pool

# project imports
from pathPlanning.Constants import planPathResultsFilename, planSettingsFilename, executeResultsFilename
from pathPlanning.RunnerUtils import loadYamlContents, toDir, getVarFromString
# from pathPlanning.PlotUtils import plotMissionTimes
from pathPlanning.EnvUtils import envFromParamsOrFile

def harvestMissionTimeInfo(s):
	"""
	Parallelizable function for harvesting mission time info from folder s
	returns [plannedMissionTime, meanMissionTime, folderName]
	"""

	plannedTime = None
	execTime = None
	endFolderName = None

	try:
		# load plan path results
		thisDir = toDir(s)
		planPathResults = loadYamlContents(thisDir, planPathResultsFilename, verbose=False)

		# look for path settings one folder above
		planFolder = os.path.split(thisDir)[0]
		planSettings = loadYamlContents(planFolder, planSettingsFilename, verbose=False)

		# construct environment
		envParams = planSettings['ENVIRONMENT']
		if isinstance(envParams, str): # this is a path to another file, not params
			envParams = os.path.join(planFolder, envParams)
		env = envFromParamsOrFile(envParams, verbose=False)

		# running sum of planned path, starting with first release
		plannedTime = env.estimateMean(
			planPathResults['ugv_point_map'][planPathResults['ugv_path'][0]],
			planPathResults['ugv_point_map'][planPathResults['ugv_path'][1]],
			'UGV'
		)
		numTours = len(planPathResults['uav_tours'])
		for i in range(numTours):
			# max tour time per agent type
			uavTime = planPathResults['tour_costs']['UAV'][i][0]
			plannedTime += max(
				uavTime,
				planPathResults['tour_costs']['UGV'][i][0]
			)
			# max of charging time or travel time to next tour
			#	note that this will wait at the final end point for 100% charge
			chargeTime = (planSettings['UAV_BATTERY_TIME'] - uavTime) / planSettings['CHARGE_RATE']
			plannedTime += max(
				chargeTime,
				env.estimateMean(
					planPathResults['ugv_point_map'][planPathResults['ugv_path'][2*i + 2]],
					planPathResults['ugv_point_map'][planPathResults['ugv_path'][2*i + 3]],
					'UGV'
				)
			)

		# mean of successful path executions
		execTime = None
		try:
			execResults = loadYamlContents(thisDir, executeResultsFilename, verbose=False)
			execTime = np.mean( [
				planSettings['UAV_BATTERY_TIME'] * len(execResults['TOUR_ATTEMPTS']) -
				sum(execResults['REMAINING_FLIGHT_TIMES'][i]) +
				sum(execResults['UGV_TRANSIT_TIMES'][i])
				for i in range(execResults['NUM_RUNS'])
				if execResults['REMAINING_FLIGHT_TIMES'][i][-1] > 0
			] )
		except Exception:
			print('Failure to load', executeResultsFilename, 'in', thisDir)
			print(traceback.format_exc())

		endFolderName = os.path.split(thisDir)[1]

	except Exception:
		print('Failure during plotting of', s)
		print(traceback.format_exc())

	return [plannedTime, execTime, endFolderName]

if __name__ == '__main__':
	# print help message if necessary
	if len(sys.argv) < 2 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python plotMissionTimes.py /path/to/results/folder/ [/path/to/results/folder2/ ...]')
		exit()

	# for each results folder, get plan_path_results time and any available execute_results times
	p = Pool()
	results = p.map(harvestMissionTimeInfo, sys.argv[1:])

	# planned time, mean exec time, folder name
	detGroup = [[],[],[]]
	stochGroupNoRefine = [[],[],[]]
	stochGroupRefine = [[],[],[]]

	# save to groups
	for r in results:
		if any(e is None for e in r):
			print('Folder type', r[2], 'not recognized')
			continue
		if 'deterministic' in r[2].lower():
			detGroup[0].append(r[0])
			detGroup[1].append(r[1])
			detGroup[2].append(r[2])
		elif 'stochastic' in r[2].lower():
			if getVarFromString(r[2], 'REFINE_TOURS') == 'True':
				stochGroupRefine[0].append(r[0])
				stochGroupRefine[1].append(r[1])
				stochGroupRefine[2].append(r[2])
			else:
				stochGroupNoRefine[0].append(r[0])
				stochGroupNoRefine[1].append(r[1])
				stochGroupNoRefine[2].append(r[2])

	# plot
	fig, ax = plt.subplots()
	if len(detGroup[0]) > 0:
		ax.plot(detGroup[0], detGroup[1], '.k', label='Deterministic')
	if len(stochGroupNoRefine[0]) > 0:
		ax.plot(stochGroupNoRefine[0], stochGroupNoRefine[1], 'vr', label='Stochastic Unrefined')
	if len(stochGroupRefine[0]) > 0:
		ax.plot(stochGroupRefine[0], stochGroupRefine[1], '^g', label='Stochastic Refined')

	plt.grid(True)
	plt.xlabel('Planned Mission Time')
	plt.ylabel('Mean Successful Mission Time')
	plt.legend()

	# group data together by common folder name contents
	def removeVariableFromFolderName(folderName, varToRemove):
		startIndex = folderName.index(varToRemove) # start of var name
		endIndex = folderName[startIndex + len(varToRemove) + 1:].index('_') if '_' in folderName[startIndex + len(varToRemove) + 1:] else len(folderName) # underscore after var value, if there is one
		return folderName[:startIndex] + folderName[startIndex + len(varToRemove) + 1 + endIndex + 1:]

	varName = 'REFINE_TOURS'
	stochGroupRefineTrimmed = [removeVariableFromFolderName(f, varName) for f in stochGroupRefine[2]]
	stochGroupNoRefineTrimmed = [removeVariableFromFolderName(f, varName) for f in stochGroupNoRefine[2]]
	pairs = [[i,j] for i in range(len(stochGroupRefineTrimmed)) for j in range(len(stochGroupNoRefineTrimmed)) if stochGroupRefineTrimmed[i] == stochGroupNoRefineTrimmed[j]]

	stochGroupRefinePaired = [[dataVec[pair[0]] for pair in pairs] for dataVec in stochGroupRefine[:2]]
	stochGroupNoRefinePaired = [[dataVec[pair[1]] for pair in pairs] for dataVec in stochGroupNoRefine[:2]]

	fig, ax = plt.subplots()
	ax.plot([stochGroupRefinePaired[0], stochGroupNoRefinePaired[0]], [stochGroupRefinePaired[1], stochGroupNoRefinePaired[1]], '-k')
	ax.plot(stochGroupNoRefinePaired[0], stochGroupNoRefinePaired[1], 'vr', label='Unrefined')
	ax.plot(stochGroupRefinePaired[0], stochGroupRefinePaired[1], '^g', label='Refined')
	plt.grid(True)
	plt.xlabel('Planned Mission Time')
	plt.ylabel('Mean Successful Mission Time')
	plt.legend()

	plt.show()
