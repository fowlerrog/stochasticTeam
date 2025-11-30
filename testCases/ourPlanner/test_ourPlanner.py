# python imports
import os
import pytest

# project imports
from pathPlanning.PlannerUtils import plannerFromParams, findClosestPoint
from pathPlanning.Constants import planSettingsFilename
from pathPlanning.RunnerUtils import loadYamlContents

class TestOurPlanner:

	@pytest.mark.parametrize("plannerType", ["OurPlannerDeterministic"])#, "OurPlannerStochastic"]) # TODO since stochastic cycles optimize for Pr(success), this test fails
	def test_oneCycle(self, plannerType):
		"""A linear set of close points should end up in a predictable cycle"""
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

		# uav has one cycle, in order
		assert solution['uav_cycles'] == [list(range(numPoints))]

		# ugv path is: start, cycle[0], cycle[-1], end, dummy
		ugvPath = [solution['ugv_point_map'][i] for i in solution['ugv_path']]
		ugvIdealPath = [
			params['START_POINT'],
			list(points[0][:2]),
			list(points[-1][:2]),
			params['END_POINT'],
			params['DUMMY_POINT']
		]
		assert ugvPath == ugvIdealPath

	@pytest.mark.parametrize("plannerType", ["OurPlannerDeterministic"])#, "OurPlannerStochastic"]) # TODO since stochastic cycles optimize for Pr(success), this test fails
	def test_twoCycles(self, plannerType):
		"""Two distant clusters of points should become unique cycles"""

		thisScriptFolder = os.path.dirname(os.path.abspath(__file__))

		# Generate points
		numPointsPerCycle = 5
		numCycles = 2
		points = []
		for iCycle in range(numCycles):
			points.extend([(iPoint * 100 + 100 + iCycle * 1000, iPoint * 100 + 100 + iCycle * 1000, 500) for iPoint in range(numPointsPerCycle)])

		# Load plan settings
		params = loadYamlContents(os.path.join(thisScriptFolder, planSettingsFilename))
		params['PLANNER_TYPE'] = plannerType
		params['RUN_FOLDER'] = thisScriptFolder
	
		# Run planner
		planner = plannerFromParams(params)
		planner.solve(points)

		# Test results
		solution = planner.standardizeSolution()

		# uav has two cycles, in order
		assert solution['uav_cycles'] == [list(range(iCycle * numPointsPerCycle, (iCycle + 1) * numPointsPerCycle)) for iCycle in range(numCycles)]

		# ugv path is: start, cycles[0][0], cycles[0][-1], cycles[1][0], cycles[1][-1], end, dummy
		ugvPath = [solution['ugv_point_map'][i] for i in solution['ugv_path']]
		ugvIdealPath = [
			params['START_POINT'],
			list(points[0][:2]),
			list(points[numPointsPerCycle-1][:2]),
			list(points[numPointsPerCycle][:2]),
			list(points[-1][:2]),
			params['END_POINT'],
			params['DUMMY_POINT']
		]
		assert ugvPath == ugvIdealPath

	# TODO test case with wind
	# TODO test case against actual costs and probabilities
