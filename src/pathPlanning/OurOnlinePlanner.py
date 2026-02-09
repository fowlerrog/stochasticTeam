
# python imports
import math
import numpy as np
import traceback

# project imports
from .OurPlanner import OurPlannerStochastic, Cost, safeExp

class OurOnlinePlanner(OurPlannerStochastic):
	"""
	Implements the general case of planning
	in which there may be different UAV and UGV start points
	there may be a nonzero current flight elapsed time
	the considered points may be a subset of the full set
	and risk may be constrained by the rest of the plan
	"""
	params = {} # input parameters
	horizonTours = None # number of tours, including the remainder of the current, to replan
	solveTime = 0 # previous solve time

	globalRiskLimit = None # total risk limit, whereas params['FAILURE_RISK'] will be overwritten for each solve
	together = None # flag for whether agents are together
	uavFlightTime = None # current uav flight time
	uavPosition = None # current uav position
	ugvPosition = None # current ugv position
	replanPoints = None # list of uav points on which to solve
	newPointOrder = None # reordering of replanPoints

	def __init__(self, params):
		"""
		Constructor:
		params = dict of parameters
		"""
		super().__init__(params)
		self.horizonTours = params['HORIZON_TOURS']
		self.globalRiskLimit = params['FAILURE_RISK']
		self.distTol = 1e-3 # points closer than this are considered the same

	def getSolveTime(self):
		"""Returns most recent solve time"""
		return self.solveTime

	def solve(self, uavTours, uavPoints, ugvOrder, ugvPoints, iTour, jTour, ugvIndex, uavPosition, ugvPosition, uavFlightTime):
		"""
		Replans some segment of the upcoming tour:
		uavTours, ugvOrder, ugvPoints, successFlag = solve(...)
		where
		uavTours = list of lists of uav point indices
		uavPoints = list of uav points
		ugvOrder = list of ugv point indices
		ugvPoints = list of ugv points
		iTour = most recent index tour index
		jTour = most recent index in most recent tour
		ugvIndex = most recent index in ugvOrder
		uavPosition = current uav position
		ugvPosition = current ugv position
		uavFlightTime = current uav flight time
		"""

		# store variables for use during solving
		self.uavFlightTime = uavFlightTime
		self.uavPosition = uavPosition
		self.ugvPosition = ugvPosition

		# logical flag for whether uav and ugv are together
		together = ugvIndex % 2 == 0 # just after collect -> together
		self.together = together

		# check if a replan is meaningful
		if iTour >= len(uavTours) - 1 and jTour >= len(uavTours[iTour]) - 2:
			self.solveTime = 0
			return uavTours, ugvOrder, ugvPoints, True

		# define planning horizon (inclusive)
		iMax = min(iTour + self.horizonTours, len(uavTours) - 1)

		# collect points
		replanIndices = [n for i in range(iTour + 1, iMax + 1) for n in uavTours[i]]
		if together: # can also replan upcoming UAV point in current tour
			replanIndices.extend([n for n in uavTours[iTour][jTour:]])
		else:
			replanIndices.extend([n for n in uavTours[iTour][jTour+1:]])
		replanIndices = list(set(replanIndices))
		replanIndices.sort()
		replanPoints = [uavPoints[i] for i in replanIndices]

		# define start and end points (which are not required to be in replanPoints)
		# start is the next uav point
		if together:
			startPoint = uavPoints[uavTours[iTour][jTour]]
		else:
			startPoint = uavPoints[uavTours[iTour][jTour + 1]]
		endPoint = uavPoints[uavTours[iMax][-1]] # last point of last replanned tour

		# find the actual risk limit by evaluating all tours past the horizon
		#	using edgeCost because constructCost would require a cost matrix
		logRiskLimit = math.log(1 - self.globalRiskLimit)
		for i in range(iMax + 1, len(uavTours)):
			ugvLogSuccessChance = self.evaluateConstraintFloat(
				self.edgeCost(ugvPoints[ugvOrder[2*i + 1]], ugvPoints[ugvOrder[2*i + 2]], 'UGV'),
				'UGV'
			)
			uavLogSuccessChance = self.evaluateConstraintFloat(
				sum([
					self.edgeCost(
						uavPoints[uavTours[i][j]],
						uavPoints[uavTours[i][j+1]],
						'UAV'
					) for j in range(len(uavTours[i]) - 1)
				], start=self.baseCost('UAV')),
			'UAV')
			logRiskLimit -= ugvLogSuccessChance + uavLogSuccessChance
		riskLimit = 1 - safeExp(logRiskLimit)
		self.logSuccessLimit = logRiskLimit
		self.params['FAILURE_RISK'] = riskLimit

		# save replanPoints for later use
		self.replanPoints = replanPoints

		# solve subset of path
		super().solve(replanPoints, startPoint=startPoint, endPoint=endPoint)

		# check for failure
		if len(self.solution) == 0:
			print('Replanning failed')
			self.solveTime = 0
			return uavTours, ugvOrder, ugvPoints, False

		# apply changes to plan
		newUavOrder = self.solution['uav_tours']
		newUgvOrder = self.solution['ugv_path']
		newUgvPoints = self.solution['ugv_point_map']

		# it is possible for the planner to duplicate a uav point if it is both the start and end of the TSP solution
		reindexedNewUavOrder = [[replanIndices[self.newPointOrder[i]] for i in tour] for tour in newUavOrder]
		reindexedNewUavOrder = [[tour[i] for i in range(len(tour)) if i == len(tour) - 1 or tour[i] != tour[i+1]] for tour in reindexedNewUavOrder] # remove sequences of repeating points

		# UAV tour changes only differ by whether the 'current' air point can be replanned
		#	because iTour,jTour refer to the most recent point during a tour
		#	and to the NEXT uav point when between tours
		if together:
			updatedUavTours = uavTours[:iTour] + [uavTours[iTour][:jTour] + reindexedNewUavOrder[0]] + reindexedNewUavOrder[1:] + uavTours[iMax+1:]
		else:
			updatedUavTours = uavTours[:iTour] + [uavTours[iTour][:jTour+1] + reindexedNewUavOrder[0]] + reindexedNewUavOrder[1:] + uavTours[iMax+1:]

		# because we change the ugv point list, we also have to be careful to re-index the order list
		updatedUgvPoints = []
		numUnchangedTours = len(uavTours) - iMax - 1
		for i in range(ugvIndex + 1): # before replan
			updatedUgvPoints.append(ugvPoints[ugvOrder[i]])
		if self.together: # replace iTour release
			updatedUgvPoints.append(newUgvPoints[newUgvOrder[1]])
		for i in range(2, len(newUgvOrder) - 1): # during replan, ignoring start and end because they are repeats
			updatedUgvPoints.append(newUgvPoints[newUgvOrder[i]])
		for i in range(len(ugvOrder) - 1 - 2*numUnchangedTours, len(ugvOrder)): # after replan
			updatedUgvPoints.append(ugvPoints[ugvOrder[i]])
		updatedUgvOrder = list(range(len(updatedUgvPoints)))

		# return
		self.solveTime = self.timeInfo['TOTAL_TIME']
		return updatedUavTours, updatedUgvOrder, updatedUgvPoints, True

	def edgeCost(self, point1, point2, agentType):
		"""Helper function to return agent costs for points outside of the cost matrix"""
		return Cost(
			self.env.estimateMean(point1, point2, agentType),
			self.env.estimateVariance(point1, point2, agentType)
		)

	def constructTourCost(self, tour, agentType=''):
		"""Checks for the special case: agents apart and first tour"""
		cost = super().constructTourCost(tour, agentType)
		if not self.together and 0 in tour:
			if agentType == 'UAV':
				cost += self.edgeCost(self.uavPosition, self.replanPoints[self.newPointOrder[tour[0]]], 'UAV') + \
				Cost(self.uavFlightTime, 0) + \
				self.baseCost('UAV') * -0.5 # TODO this is approximate: landing = 0.5 base cost
			elif agentType == 'UGV':
				cost = self.edgeCost(
					self.project(self.ugvPosition),
					self.project(self.replanPoints[self.newPointOrder[tour[-1]]]),
					'UGV') + \
				Cost(self.uavFlightTime, 0)
			else:
				raise IndexError('agentType %s not recognized'%agentType)
		return cost

	def solveUavGlobalTsp(self, uavPoints, startPoint, endPoint):
		"""Helper function to save point reordering map, for use during partitioning"""
		newUavPoints, newPointOrder = super().solveUavGlobalTsp(uavPoints, startPoint, endPoint)
		self.newPointOrder = newPointOrder
		return newUavPoints, newPointOrder

	def logSuccessChanceUavUgv(self, uavCost, tourStartIndex, tourEndIndex):
		"""
		Returns log chance of both uav and ugv cost not exceeding the limit
		by sweeping collect points
		Note that because this is written for use with the PartitionSolver, which has n+1 costs, tourStartIndex is inclusive and tourEndIndex is exclusive
		"""
		# check for the usual case
		if self.together or tourStartIndex > 0:
			return super().logSuccessChanceUavUgv(uavCost, tourStartIndex, tourEndIndex)

		# otherwise, we are considering a tour in progress
		#	for which release has already passed
		#	and we already have some nonzero flight time

		baseTour = list(range(tourStartIndex, tourEndIndex))
		if len(baseTour) == 1:
			possibleTours = [baseTour]
		else:
			possibleTours = [
				baseTour[:i] + baseTour[i+1:] + [baseTour[i]]
				for i in range(1, len(baseTour))
			] # we do not allow the immediate point to be moved to the end of the tour because this causes a duplication issue e.g tour = [A B A] -> [B A B] - > [A B A] under repeated replanning

		logSuccessChances = [
			self.evaluateConstraintFloat(
				self.constructTourCost(t, 'UAV'),
				'UAV' ) + # UAV log success chance
			self.evaluateConstraintFloat(
				self.constructTourCost(t, 'UGV'),
				'UGV' ) # plus UGV log success chance
			for t in possibleTours
		]

		bestCollect = np.argmax(logSuccessChances)
		bestLogChance = logSuccessChances[bestCollect] # best log success chance

		# chance, (release global index, collect global index)
		return bestLogChance, (tourStartIndex, possibleTours[bestCollect][-1])
