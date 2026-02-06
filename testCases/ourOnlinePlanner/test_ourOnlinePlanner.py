# python imports
import os
import pytest
import random
from scipy.spatial.distance import euclidean

# project imports
from pathPlanning.PlannerUtils import plannerFromParams
from pathPlanning.Constants import planSettingsFilename
from pathPlanning.RunnerUtils import loadYamlContents
from pathPlanning.NodeUtils import generatePoints

class TestOurOnlinePlanner:

	@pytest.mark.parametrize("randomSeed", range(5))
	@pytest.mark.parametrize("numPoints", [25])
	@pytest.mark.parametrize("numReplans", [10])
	@pytest.mark.parametrize("together", [True, False])
	def test_ourOnlinePlanner(self, randomSeed, numPoints, numReplans, together):
		"""
		Stochastic online planner should:
		not change UAV or UGV final orders before the current time or after the horizon time
		not change UAV point list
		not violate risk allowance
		"""

		thisScriptFolder = os.path.dirname(os.path.abspath(__file__))
		distTol = 1e-3 # two points match within this distance
		timeTol = 1e-3 # two times match within this time
		onlinePlannerFilename = 'online_planner_settings.yaml'

		# Load plan settings
		planParams = loadYamlContents(os.path.join(thisScriptFolder, planSettingsFilename))

		# Generate points
		random.seed(randomSeed)
		points = generatePoints(planParams['POINTS'] | {'NUM_POINTS' : numPoints})
	
		# Run planner
		planner = plannerFromParams(planParams['PLANNER'])
		planner.solve(points, startPoint = planParams['START_POINT'], endPoint = planParams['END_POINT'])
		solution = planner.standardizeSolution()

		# Load results
		originalUavPoints = solution['uav_points']
		originalUavTours = solution['uav_tours']
		originalUgvPoints = solution['ugv_point_map']
		originalUgvOrder = solution['ugv_path']

		# Construct online planner
		onlinePlanParams = loadYamlContents(os.path.join(thisScriptFolder, onlinePlannerFilename))
		onlinePlanner = plannerFromParams(onlinePlanParams)

		# For each replan
		for _ in range(numReplans):
			# Choose a current state
			iTour = random.randint(0, len(originalUavTours) - 1)
			if together: # just after collect during tour iTour-1
				ugvIndex = iTour * 2
				jTour = 0
				ugvPosition = originalUgvPoints[originalUgvOrder[ugvIndex]]
				uavPosition = ugvPosition
				uavFlightTime = 0
			else: # just after point iTour,jTour
				ugvIndex = iTour * 2 + 1
				jTour = random.randint(0, len(originalUavTours[iTour]) - 2)
				uavPosition = originalUavPoints[originalUavTours[iTour][jTour]]
				# this could be varied, but we would have to be careful that there is still a valid plan to be found
				#	so we're just going to use mean times
				uavFlightTime = planner.env.estimateMean(
					[*originalUgvPoints[originalUgvOrder[ugvIndex]], 0],
					originalUavPoints[originalUavTours[iTour][0]],
					'UAV'
				) # takeoff
				for j in range(jTour):
					uavFlightTime += planner.env.estimateMean(
						originalUavPoints[originalUavTours[iTour][j]],
						originalUavPoints[originalUavTours[iTour][j+1]],
						'UAV'
					)
				# propagate UGV position
				predictedUgvTime = planner.env.estimateMean(
					originalUgvPoints[originalUgvOrder[ugvIndex]],
					originalUgvPoints[originalUgvOrder[ugvIndex + 1]],
					'UGV'
				)
				if uavFlightTime >= predictedUgvTime:
					ugvPosition = originalUgvPoints[originalUgvOrder[ugvIndex + 1]]
				else:
					ugvPosition = [
						originalUgvPoints[originalUgvOrder[ugvIndex]][i] +
						(originalUgvPoints[originalUgvOrder[ugvIndex + 1]][i] - originalUgvPoints[originalUgvOrder[ugvIndex]][i]) * uavFlightTime / predictedUgvTime
						for i in range(2)
					]
					ugvPosition = [*ugvPosition, 0]

			# Calculate replan			
			newUavTours, newUgvOrder, newUgvPoints = onlinePlanner.solve(
				originalUavTours, originalUavPoints, originalUgvOrder, originalUgvPoints,
				iTour, jTour, ugvIndex, uavPosition, ugvPosition, uavFlightTime
			)

			# Test results:
			numReplanTours = onlinePlanParams['HORIZON_TOURS']
			tourPlanHorizon = min(iTour + numReplanTours, len(originalUavTours) - 1) # inclusive
			numUnchangedTours = len(originalUavTours) - tourPlanHorizon - 1

			# UAV points before and after plan are unchanged
			assert all([
				len(newTour) == len(oldTour) and newTour == oldTour
				for newTour, oldTour in zip(newUavTours[:iTour], originalUavTours[:iTour])
			])
			if together:
				assert all([
					newPoint == oldPoint
					for newPoint, oldPoint in zip(newUavTours[iTour][:jTour], originalUavTours[iTour][:jTour])
				])
			else:
				assert all([
					newPoint == oldPoint
					for newPoint, oldPoint in zip(newUavTours[iTour][:jTour+1], originalUavTours[iTour][:jTour+1])
				])
			if numUnchangedTours > 0:
				assert all([
					len(newTour) == len(oldTour) and newTour == oldTour
					for newTour, oldTour in zip(newUavTours[-numUnchangedTours:], originalUavTours[-numUnchangedTours:])
				])

			# UGV points before and after plan are unchanged
			assert all([
				euclidean(newUgvPoints[newInd], originalUgvPoints[oldInd]) < distTol
				for newInd, oldInd in zip(newUgvOrder[:ugvIndex+1], originalUgvOrder[:ugvIndex+1])
			])
			assert all([
				euclidean(newUgvPoints[newInd], originalUgvPoints[oldInd]) < distTol
				for newInd, oldInd in zip(newUgvOrder[-1 - 2*numUnchangedTours:], originalUgvOrder[-1 - 2*numUnchangedTours:])
			])

			# no change in UAV point list
			#	because we only return an index ordering and not a new list of points,
			#	this can only be violated by skipping points
			allOriginalUavIndices = {n for tour in originalUavTours for n in tour}
			allNewUavIndices = {n for tour in newUavTours for n in tour}
			assert allOriginalUavIndices == allNewUavIndices

			# no violation in risk allowance
			logProbSuccess = 0
			# TODO
