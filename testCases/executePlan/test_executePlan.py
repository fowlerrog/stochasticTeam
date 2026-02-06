# python imports
import os
import pytest
import random
from scipy.spatial.distance import euclidean

# project imports
from pathPlanning.PlannerUtils import plannerFromParams
from pathPlanning.Constants import planSettingsFilename, executeSettingsFilename
from pathPlanning.RunnerUtils import loadYamlContents
from pathPlanning.NodeUtils import generatePoints
from pathPlanning.ExecuteUtils import executePlanFromParamsWithOnlinePlanner, calculatePlannedMission

class TestExecutePlan:

	@pytest.mark.parametrize("randomSeed", range(5))
	@pytest.mark.parametrize("numPoints", [15])
	def test_executePlan(self, randomSeed, numPoints):
		"""
		Stochastic planner tour execution with no online planner should:
		not change UAV or UGV final orders
		not change UGV final point list
		match tour times when given a 0-variance environment
		match calculate planned total mission time
		"""

		thisScriptFolder = os.path.dirname(os.path.abspath(__file__))
		planResultsFolder = 'results_from_planning'
		distTol = 1e-3 # two points match within this distance
		timeTol = 1e-3 # two times match within this time

		# Load plan settings
		planParams = loadYamlContents(os.path.join(thisScriptFolder, planSettingsFilename))

		# Generate points
		random.seed(randomSeed)
		points = generatePoints(planParams['POINTS'] | {'NUM_POINTS' : numPoints})
	
		# Run planner
		planner = plannerFromParams(planParams['PLANNER'] | {'RUN_FOLDER' : thisScriptFolder, 'SAVE_PATH_FOLDER' : planResultsFolder})
		planner.solve(points, startPoint = planParams['START_POINT'], endPoint = planParams['END_POINT'])
		solution = planner.standardizeSolution()
		planner.printResultsToYaml()

		# Evaluate mission plan
		totalPlannedMeanTime, totalPlannedLogProbSuccess = calculatePlannedMission(planParams, solution)

		# Execute mission plan 1 time with 0 variance
		executeParams = loadYamlContents(os.path.join(thisScriptFolder, executeSettingsFilename))
		executeParams['NUM_RUNS'] = 1
		executeParams['ENVIRONMENT']['AGENT_ENVIRONMENTS']['UAV']['WEIGHT_STD_DEV'] = 0
		executeParams['ENVIRONMENT']['AGENT_ENVIRONMENTS']['UGV']['WEIGHT_STD_DEV'] = 0
		executeResults = executePlanFromParamsWithOnlinePlanner(
			executeParams,
			os.path.join(thisScriptFolder, planSettingsFilename),
			os.path.join(thisScriptFolder, planResultsFolder)
		)

		# Test results:
		assert executeResults['NUM_RUNS'] == 1

		# no failures
		assert executeResults['TOUR_ATTEMPTS'] == len(executeResults['UAV_FINAL_TOURS'][0])

		# same number of tours
		assert len(solution['uav_tours']) == len(executeResults['UAV_FINAL_TOURS'][0])

		# uav tours are unchanged
		assert all(
			len(tour1) == len(tour2) and tour1 == tour2
			for tour1, tour2 in zip(solution['uav_tours'], executeResults['UAV_FINAL_TOURS'][0])
		)

		# ugv tours are unchanged
		assert all(
			euclidean([*solution['ugv_point_map'][ugvPoint1], 0], executeResults['UGV_FINAL_POINTS'][0][ugvPoint2]) < distTol
			for ugvPoint1, ugvPoint2 in zip(solution['ugv_path'], executeResults['UGV_FINAL_ORDERS'][0])
		)

		# each tour execution time matches planned mean time
		plannedMeanTourTimes = [
			max( [solution['tour_costs'][agentType][iTour][0] for agentType in solution['tour_costs'].keys()] )
			for iTour in range(len(solution['uav_tours']))
		]
		assert all(
			abs(executeResults['TOUR_TIMES'][0][iTour] - plannedMeanTourTimes[iTour]) < timeTol
			for iTour in range(len(solution['uav_tours']))
		)

		# execution time matches total planned mean time
		assert abs(executeResults['TOTAL_TIMES'][0] - totalPlannedMeanTime) < timeTol
