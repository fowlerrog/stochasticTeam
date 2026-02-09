# python imports
import sys
import os
from random import seed
from copy import deepcopy
import matplotlib.pyplot as plt

# project imports
from pathPlanning.Constants import executeSettingsFilename, planSettingsFilename, planPathResultsFilename
from pathPlanning.RunnerUtils import loadYamlContents
from pathPlanning.EnvUtils import envFromParams
from pathPlanning.PlannerUtils import plannerFromParams
from pathPlanning.ExecuteUtils import stepOnlineExecution
from pathPlanning.PlotUtils import plotTours

# this script executes a plan,
#	pausing to display after each step
# based on executePlanFromParamsWithOnlinePlanner()

def plotPlanWithPositions(thisUavTours, uavPoints, thisUgvPoints, thisUgvOrder, uavPosition, ugvPosition, fig, ax):
	print(f'Plotting: UAV {uavPosition}\tUGV {ugvPosition}')
	if ax is not None:
		ax.clear()
	fig, ax = plotTours(thisUavTours, uavPoints, thisUgvPoints, thisUgvOrder, show=False, fig=fig, ax=ax)
	ax.plot(ugvPosition[0], ugvPosition[1], 'rv', markersize=16)
	ax.plot(uavPosition[0], uavPosition[1], 'g^', markersize=16)
	plt.draw()
	plt.pause(0.1)
	input('Press Enter to continue')

if __name__ == '__main__':
	# print help message if necessary
	if len(sys.argv) < 4 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python runOnlinePlanner.py /path/to/%s /path/to/%s /path/to/%s'%(executeSettingsFilename, planSettingsFilename, planPathResultsFilename))
		exit()

	executeParams = loadYamlContents(sys.argv[1], executeSettingsFilename)
	planSettingsPath = sys.argv[2]
	planResultsPath = sys.argv[3]
	verbose = True

	# load parameters from yamls
	absPlanSettingsPath = os.path.abspath(planSettingsPath)
	planParams = loadYamlContents(absPlanSettingsPath, planSettingsFilename)
	absResultsPath = os.path.abspath(planResultsPath)
	resultsDict = loadYamlContents(absResultsPath, planPathResultsFilename)

	# construct environment
	envParams = executeParams['ENVIRONMENT']
	if verbose:
		print("Constructing environment:", envParams)
	env = envFromParams(envParams)

	# construct online planner
	onlinePlanner = None
	if 'ONLINE_PLANNER' in executeParams:
		onlineParams = executeParams['ONLINE_PLANNER']
		if verbose:
			print("Constructing online planner:", onlineParams)
		onlinePlanner = plannerFromParams(onlineParams)

	# parse path
	uavPoints = resultsDict['uav_points']
	uavTours = resultsDict['uav_tours']
	ugvOrder = resultsDict['ugv_path']
	ugvPoints = resultsDict['ugv_point_map']

	# TODO perhaps this should be in its own agent definition file, or the environment?
	uavMaxTime = planParams['PLANNER']['UAV_BATTERY_TIME']
	uavChargeRate = planParams['PLANNER']['CHARGE_RATE']

	# set up RNG
	if 'SEED' in executeParams and executeParams['SEED'] is not None:
		seed(executeParams['SEED'])

	# start run
	thisUavTours = deepcopy(uavTours)
	thisUgvOrder = deepcopy(ugvOrder)
	thisUgvPoints = deepcopy(ugvPoints)

	ugvIndex = 0 # most recent ugvOrder position
	ugvPosition = None # actual ugv 3d position
	iTour = 0 # most recent position in uavTours, or the upcoming tour if between tours
	jTour = 0 # most recent position in uavTours[i], or the upcoming tour if between tours
	uavTourTime = 0 # how long the UAV has been in flight (depleting its battery)

	# plot offline plan
	fig, ax = plt.subplots(figsize=(8, 6)) # create figure
	plt.ion() # interactive mode on
	plt.show(block=False) # show figure
	plotPlanWithPositions(thisUavTours, uavPoints, thisUgvPoints, thisUgvOrder, thisUgvPoints[thisUgvOrder[0]], thisUgvPoints[thisUgvOrder[0]], fig, ax)

	# note that every instance of this loop occurs just after we are AT ugvIndex, iTour, jTour
	#	and that we cannot change the plan for those points because they have occurred
	timeoutFailure = 0
	while ugvIndex < len(thisUgvOrder) - 1:
		thisUavTours, thisUgvOrder, thisUgvPoints, \
		ugvIndex, ugvPosition, iTour, jTour, uavTourTime, \
		_, _, _, \
		timeoutFailure, _, _ = \
		stepOnlineExecution(onlinePlanner, env, uavPoints,
			thisUavTours, thisUgvOrder, thisUgvPoints,
			ugvIndex, ugvPosition, iTour, jTour, uavTourTime,
			[], 0, [],
			timeoutFailure, 0, 0,
			uavChargeRate, uavMaxTime,
			verbose=verbose
		)

		# pause to plot
		if ugvIndex % 2 == 0:
			plotPlanWithPositions(thisUavTours, uavPoints, thisUgvPoints, thisUgvOrder, thisUgvPoints[thisUgvOrder[ugvIndex]], thisUgvPoints[thisUgvOrder[ugvIndex]], fig, ax)
		elif ugvIndex == len(thisUgvOrder) - 1:
			break
		else:
			plotPlanWithPositions(thisUavTours, uavPoints, thisUgvPoints, thisUgvOrder, uavPoints[thisUavTours[iTour][jTour]], ugvPosition, fig, ax)

		if timeoutFailure:
			print(f'TIMEOUT FAILURE @ t_uav = {uavTourTime:.2f}')
			break

	print('###############')
	print('Final plan:')
	print(thisUgvOrder)
	print(thisUgvPoints)
	print(len(uavPoints), 'uav points')
	print(thisUavTours)
	print('###############')

	for i in range(0, len(thisUgvOrder), 2):
		print(f'UGV: {thisUgvPoints[thisUgvOrder[i]]}\n     {thisUgvPoints[thisUgvOrder[i+1]]}')
		if i < len(thisUgvOrder) - 3:
			print(f'UAV: ' + '\n     '.join(f'{uavPoints[p]}' for p in thisUavTours[i // 2]))
