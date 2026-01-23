# python imports
import os
import pytest
import random
from scipy.spatial.distance import euclidean
import numpy as np

# project imports
from pathPlanning.PlannerUtils import plannerFromParams
from pathPlanning.Constants import planSettingsFilename
from pathPlanning.RunnerUtils import loadYamlContents
from pathPlanning.NodeUtils import generatePoints

class TestOurPlanner:

	@pytest.mark.parametrize("plannerType", ["OurPlannerDeterministic"])#, "OurPlannerStochastic"]) # TODO since stochastic tours optimize for Pr(success), this test fails
	def test_oneTour(self, plannerType):
		"""A linear set of close points should end up in a predictable tour"""
		thisScriptFolder = os.path.dirname(os.path.abspath(__file__))

		# Generate points
		numPoints = 5
		points = [(iPoint * 100 + 100, iPoint * 100 + 100, 500) for iPoint in range(numPoints)]

		# Load plan settings
		params = loadYamlContents(os.path.join(thisScriptFolder, planSettingsFilename))
		params['PLANNER_TYPE'] = plannerType
		params['RUN_FOLDER'] = thisScriptFolder
	
		# Run planner
		planner = plannerFromParams(params)
		planner.solve(points)

		# Test results
		solution = planner.standardizeSolution()

		# uav has one tour, in order
		assert solution['uav_tours'] == [list(range(numPoints))]

		# ugv path is: start, tour[0], tour[-1], end, dummy
		ugvPath = [solution['ugv_point_map'][i] for i in solution['ugv_path']]
		ugvIdealPath = [
			params['START_POINT'],
			list(points[0][:2]),
			list(points[-1][:2]),
			params['END_POINT']
		]
		assert ugvPath == ugvIdealPath

	@pytest.mark.parametrize("plannerType", ["OurPlannerDeterministic"])#, "OurPlannerStochastic"]) # TODO since stochastic tours optimize for Pr(success), this test fails
	def test_twoTours(self, plannerType):
		"""Two distant clusters of points should become unique tours"""

		thisScriptFolder = os.path.dirname(os.path.abspath(__file__))

		# Generate points
		numPointsPerTour = 5
		numTours = 2
		points = []
		for iTour in range(numTours):
			points.extend([(iPoint * 100 + 100 + iTour * 1000, iPoint * 100 + 100 + iTour * 1000, 500) for iPoint in range(numPointsPerTour)])

		# Load plan settings
		params = loadYamlContents(os.path.join(thisScriptFolder, planSettingsFilename))
		params['PLANNER_TYPE'] = plannerType
		params['RUN_FOLDER'] = thisScriptFolder
	
		# Run planner
		planner = plannerFromParams(params)
		planner.solve(points)

		# Test results
		solution = planner.standardizeSolution()

		# uav has two tours, in order
		assert solution['uav_tours'] == [list(range(iTour * numPointsPerTour, (iTour + 1) * numPointsPerTour)) for iTour in range(numTours)]

		# ugv path is: start, tours[0][0], tours[0][-1], tours[1][0], tours[1][-1], end, dummy
		ugvPath = [solution['ugv_point_map'][i] for i in solution['ugv_path']]
		ugvIdealPath = [
			params['START_POINT'],
			list(points[0][:2]),
			list(points[numPointsPerTour-1][:2]),
			list(points[numPointsPerTour][:2]),
			list(points[-1][:2]),
			params['END_POINT']
		]
		assert ugvPath == ugvIdealPath

	@pytest.mark.parametrize("randomSeed", range(5))
	@pytest.mark.parametrize("numPoints", [10, 20])
	def test_tourRefinement(self, randomSeed, numPoints):
		"""
		Stochastic planner tour refinement should:
		not change number of tours
		keep release, collect points unchanged
		not remove any uav points
		not increase risk
		not increase time
		"""

		thisScriptFolder = os.path.dirname(os.path.abspath(__file__))
		distTol = 1e-3 # two points match within this distance

		# Load plan settings
		params = loadYamlContents(os.path.join(thisScriptFolder, planSettingsFilename))
		params['PLANNER_TYPE'] = 'OurPlannerStochastic'

		# Generate points
		random.seed(randomSeed)
		points = generatePoints(numPoints,
						  xRange=(0,params['SPACE_SIZE']),
						  yRange=(0,params['SPACE_SIZE']),
						  fixedZ=params['FIXED_Z'])
	
		# Run planner with and without refinement
		planner1 = plannerFromParams(params | {'REFINE_TOURS' : True})
		planner1.solve(points)
		planner2 = plannerFromParams(params | {'REFINE_TOURS' : False})
		planner2.solve(points)

		# Test results
		solution1 = planner1.standardizeSolution()
		solution2 = planner2.standardizeSolution()

		# same number of tours
		assert len(solution1['uav_tours']) == len(solution2['uav_tours'])

		# release and collect points are unchanged (uav side)
		assert all(
			euclidean(solution1['uav_points'][tour1[0]], solution2['uav_points'][tour2[0]]) < distTol and
			euclidean(solution1['uav_points'][tour1[-1]], solution2['uav_points'][tour2[-1]]) < distTol
			for tour1, tour2 in zip(solution1['uav_tours'], solution1['uav_tours'])
		)

		# release and collect points are unchanged (ugv side)
		assert all(
			euclidean(solution1['ugv_point_map'][ugvPoint1], solution2['ugv_point_map'][ugvPoint2]) < distTol
			for ugvPoint1, ugvPoint2 in zip(solution1['ugv_path'], solution1['ugv_path'])
		)

		# same uav points
		assert len(solution1['uav_points']) == len(solution2['uav_points'])
		assert all(
			any(euclidean(uavPoint1, uavPoint2) < distTol for uavPoint2 in solution2['uav_points'])
			for uavPoint1 in solution1['uav_points']
		)

		# total log prob success does not decrease
		assert sum(sum(tourLogProbs) for tourLogProbs in solution1['tour_constraint_values'].values()) >= sum(sum(tourLogProbs) for tourLogProbs in solution2['tour_constraint_values'].values())

		# uav mean time for each tour does not increase
		assert all( cost1[0] <= cost2[0] for cost1, cost2 in zip(solution1['tour_costs']['UAV'], solution2['tour_costs']['UAV']) )

	@pytest.mark.parametrize("randomSeed", range(5))
	@pytest.mark.parametrize("plannerType", ["OurPlannerDeterministic", "OurPlannerStochastic"])
	@pytest.mark.parametrize("environment", [None, "cost_matrix_env_settings.yaml"])
	def test_costMatrix(self, randomSeed, plannerType, environment):
		"""
		Tests whether OurPlanner can correctly produce and reorder a cost matrix from an environment
		"""

		thisScriptFolder = os.path.dirname(os.path.abspath(__file__))
		costTol = 1e-3 # two costs match within this tolerance
		numPoints = 10 # number of points in matrix
		agentTypes = ['UAV', 'UGV'] # agent types in environment

		# Load plan settings
		params = loadYamlContents(os.path.join(thisScriptFolder, 'cost_matrix_plan_settings.yaml'))
		params['PLANNER_TYPE'] = plannerType
		if not environment is None:
			params['ENVIRONMENT'] = environment
		params['RUN_FOLDER'] = thisScriptFolder

		# Generate points
		random.seed(randomSeed)
		points = generatePoints(numPoints,
						  xRange=(0,params['SPACE_SIZE']),
						  yRange=(0,params['SPACE_SIZE']),
						  fixedZ=params['FIXED_Z'])

		planner = plannerFromParams(params)
		for agentType in agentTypes:
			# Generate planner cost matrices
			planner.createCostMatrix(points, agentType)

			# Test mean cost matrices
			costMatrix = np.zeros((numPoints, numPoints))
			if environment is None:
				for i in range(numPoints):
					for j in range(numPoints):
						costMatrix[i][j] = euclidean(points[i], points[j])
			else:
				for i in range(numPoints):
					for j in range(numPoints):
						costMatrix[i][j] = planner.env.estimateMean(points[i], points[j], agentType)

			assert np.all(np.abs(planner.costMatrix[agentType] - costMatrix) < costTol)

			# Test var cost matrices
			if plannerType == 'OurPlannerStochastic':
				costVarMatrix = np.zeros((numPoints, numPoints))
				if environment is None:
					pass # should be zeros
				else:
					for i in range(numPoints):
						for j in range(numPoints):
							costVarMatrix[i][j] = planner.env.estimateVariance(points[i], points[j], agentType)

				assert np.all(np.abs(planner.costVarMatrix[agentType] - costVarMatrix) < costTol)

			# Test reordering
			newOrder = list(range(numPoints))
			random.shuffle(newOrder)
			newPoints = [points[i] for i in newOrder]

			# Generate planner cost matrices
			planner.reorderCostMatrix(newOrder, agentType)

			# Test mean cost matrices
			costMatrix = np.zeros((numPoints, numPoints))
			if environment is None:
				for i in range(numPoints):
					for j in range(numPoints):
						costMatrix[i][j] = euclidean(newPoints[i], newPoints[j])
			else:
				for i in range(numPoints):
					for j in range(numPoints):
						costMatrix[i][j] = planner.env.estimateMean(newPoints[i], newPoints[j], agentType)

			assert np.all(np.abs(planner.costMatrix[agentType] - costMatrix) < costTol)

			# Test var cost matrices
			if plannerType == 'OurPlannerStochastic':
				costVarMatrix = np.zeros((numPoints, numPoints))
				if environment is None:
					pass # should be zeros
				else:
					for i in range(numPoints):
						for j in range(numPoints):
							costVarMatrix[i][j] = planner.env.estimateVariance(newPoints[i], newPoints[j], agentType)

				assert np.all(np.abs(planner.costVarMatrix[agentType] - costVarMatrix) < costTol)

	# TODO test case with wind
	# TODO test case against actual costs and probabilities
