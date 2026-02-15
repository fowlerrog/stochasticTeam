# python imports
import os
import pytest
import random
from scipy.spatial.distance import euclidean
import math

# project imports
from pathPlanning.PlannerUtils import plannerFromParams
from pathPlanning.Constants import planSettingsFilename
from pathPlanning.RunnerUtils import loadYamlContents
from pathPlanning.NodeUtils import generatePoints
from pathPlanning.OurPlanner import Cost

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
		riskTol = 1e-6 # two risks match within this ratio
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
			newUavTours, newUgvOrder, newUgvPoints, successFlag = onlinePlanner.solve(
				originalUavTours, originalUavPoints, originalUgvOrder, originalUgvPoints,
				iTour, jTour, ugvIndex, uavPosition, ugvPosition, uavFlightTime
			)

			# If the planner failed there should be no change
			if not successFlag:
				assert newUavTours == originalUavTours
				assert newUgvOrder == originalUgvOrder
				assert newUgvPoints == originalUgvPoints
				continue

			# Test results:
			numReplanTours = onlinePlanParams['HORIZON_TOURS']
			tourPlanHorizon = min(iTour + numReplanTours, len(originalUavTours) - 1) # inclusive
			numUnchangedTours = len(originalUavTours) - tourPlanHorizon - 1

			# no half-tours
			assert len(newUgvOrder) % 2 == 0

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

			# UAV tours match UGV points
			assert all([
				euclidean(
					onlinePlanner.project(originalUavPoints[newUavTours[i][0]]),
					onlinePlanner.project(newUgvPoints[newUgvOrder[2*i+1]])
				) < distTol and
				euclidean(
					onlinePlanner.project(originalUavPoints[newUavTours[i][-1]]),
					onlinePlanner.project(newUgvPoints[newUgvOrder[2*i+2]])
				) < distTol
				for i in range(len(newUavTours))
			])

			# if jTour is late enough, there will not have been a replan
			#	and there will be no updates to onlinePlanner.solution
			if iTour >= len(originalUavTours) - 1 and jTour >= len(originalUavTours[iTour]) - 2:
				continue

			# no violation in risk allowance according to planner
			logProbSuccess = 0
			for tourLogProbs in onlinePlanner.solution['tour_constraint_values'].values(): # new tours
				logProbSuccess += sum(tourLogProbs)
			if numUnchangedTours > 0:
				for tourLogProbs in planner.solution['tour_constraint_values'].values(): # remaining tours
					logProbSuccess += sum(tourLogProbs[-numUnchangedTours:])
			assert logProbSuccess > math.log(1 - planParams['PLANNER']['FAILURE_RISK'])

			def assertFullTourSuccessChance(i):
				# helper function to assert log prob success of tour is correct
				#	where i=0 for consideration of iTour
				ugvLogSuccessChance = onlinePlanner.evaluateConstraintFloat(
					onlinePlanner.edgeCost(
						newUgvPoints[newUgvOrder[2*(i + iTour) + 1]],
						newUgvPoints[newUgvOrder[2*(i + iTour) + 2]],
						'UGV'),
					'UGV'
				)
				assert abs(ugvLogSuccessChance - onlinePlanner.solution['tour_constraint_values']['UGV'][i]) < riskTol

				uavLogSuccessChance = onlinePlanner.evaluateConstraintFloat(
					sum([
						onlinePlanner.edgeCost(
							originalUavPoints[newUavTours[i + iTour][j]],
							originalUavPoints[newUavTours[i + iTour][j+1]],
							'UAV')
						for j in range(len(newUavTours[i + iTour]) - 1)
					], start=onlinePlanner.baseCost('UAV')),
				'UAV')
				assert abs(uavLogSuccessChance - onlinePlanner.solution['tour_constraint_values']['UAV'][i]) < riskTol

			# risk is calculated correctly for remaining recalculated tours
			if together: # consider iTour as a full tour
				for i in range(len(newUavTours) - iTour):
					if i < numReplanTours:
						assertFullTourSuccessChance(i)
			else: # consider iTour as a partial tour
				for i in range(1, len(newUavTours) - iTour):
					if i < numReplanTours:
						assertFullTourSuccessChance(i)

				# consider the partial tour
				ugvLogSuccessChance = onlinePlanner.evaluateConstraintFloat(
					onlinePlanner.edgeCost(
						onlinePlanner.project(ugvPosition),
						onlinePlanner.project(newUgvPoints[newUgvOrder[2*iTour + 2]]),
						'UGV') +
						Cost(uavFlightTime, 0),
					'UGV'
				)
				assert abs(ugvLogSuccessChance - onlinePlanner.solution['tour_constraint_values']['UGV'][0]) < riskTol

				uavLogSuccessChance = onlinePlanner.evaluateConstraintFloat(
					sum([
						onlinePlanner.edgeCost(
							originalUavPoints[newUavTours[iTour][j]],
							originalUavPoints[newUavTours[iTour][j+1]],
							'UAV')
						for j in range(jTour, len(newUavTours[iTour]) - 1)
					], start = onlinePlanner.baseCost('UAV') * 0.5 +
						onlinePlanner.edgeCost(
							uavPosition,
							originalUavPoints[newUavTours[iTour][jTour]],
							'UAV'
						) +
						Cost(uavFlightTime, 0)
					),
				'UAV')
				assert abs(uavLogSuccessChance - onlinePlanner.solution['tour_constraint_values']['UAV'][0]) < riskTol
