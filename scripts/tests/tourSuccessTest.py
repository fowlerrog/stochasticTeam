# python imports
import random
import traceback
import numpy as np
import matplotlib.pyplot as plt

# project imports
from pathPlanning.PlannerUtils import plannerFromParams
from pathPlanning.NodeUtils import generatePoints
from pathPlanning.OurPlanner import safeExp

if __name__ == '__main__':
	# create stochastic planner 
	params = {
		'PLANNER_TYPE' : 'OurPlannerStochastic',
		'ENVIRONMENT' : {
			'TYPE' : 'SplitEnvironment',
			'AGENT_ENVIRONMENTS' : {
				'UAV' : {
					'TYPE' : 'GaussianEnvironment',
					'WEIGHT' : 0.1,
					'WEIGHT_STD_DEV' : 0.01
				},
				'UGV' : {
					'TYPE' : 'GaussianEnvironment',
					'WEIGHT' : 0.4,
					'WEIGHT_STD_DEV' : 0.04
				}
			}
		},

		'TSP_LOCAL_SEARCH' : False,
		'FAILURE_RISK' : 0.0,
		'REFINE_TOURS' : False,

		'START_POINT' : [0, 0],
		'END_POINT' : [4000, 4000],
		'DUMMY_POINT' : [4000, 0],
		'SPACE_SIZE' : 4000,
		'FIXED_Z' : 500,

		'TAKEOFF_LANDING_TIME' : 100.0,
		'UAV_BATTERY_TIME' : 600.0
	}
	planner = plannerFromParams(params)

	# generate some points
	seed = 0
	numPoints = 25
	random.seed(seed)
	points = generatePoints(numPoints,
		xRange=(0,params['SPACE_SIZE']),
		yRange=(0,params['SPACE_SIZE']),
		fixedZ=params['FIXED_Z']
	)

	# ask the planner to solve
	#	FAILURE_RISK = 0.0, so this will fail
	try:
		planner.solve(points)
	except Exception:
		print('Failure while planning:')
		print(traceback.format_exc())

	# pull the planner's partition solver data
	dpSolver = planner.partitionSolver
	totalPs = np.array(list(map(safeExp, dpSolver.DP[:,-1])))
	print(totalPs)

	probDiff = np.diff(totalPs)
	print('Min delta Prob', np.min(probDiff))
	print('\twith', np.sum(probDiff < 0), '< 0')

	# plot
	fig, ax = plt.subplots()
	ax.plot(range(len(totalPs)), totalPs, '-x')
	plt.xlabel('Number of Tours')
	plt.ylabel('Total Probability of Success')

	plt.show()
