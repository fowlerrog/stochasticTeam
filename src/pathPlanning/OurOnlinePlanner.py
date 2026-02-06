
# python imports
import math
from scipy.spatial.distance import euclidean

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
		uavTours, ugvOrder, ugvPoints = solve(...)
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
			return uavTours, ugvOrder, ugvPoints

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
		startPoint = uavPosition
		endPoint = ugvPoints[ugvOrder[3 + 2*iMax]] # release point of next tour, or end point

		# find the actual risk limit by evaluating all tours past the horizon
		logRiskLimit = math.log(1 - self.globalRiskLimit)
		for i in range(iMax + 1, len(uavTours)):
			ugvLogSuccessChance = self.evaluateConstraintFloat(
				Cost(
					self.env.estimateMean(ugvPoints[ugvOrder[2*i + 1]], ugvPoints[ugvOrder[2*i + 2]], 'UGV'),
					self.env.estimateVariance(ugvPoints[ugvOrder[2*i + 1]], ugvPoints[ugvOrder[2*i + 2]], 'UGV')
				),
			'UGV')
			uavLogSuccessChance = self.evaluateConstraintFloat(
				sum([
					Cost(
						self.env.estimateMean(uavPoints[uavTours[i][j]], uavPoints[uavTours[i][j+1]], 'UAV'),
						self.env.estimateVariance(uavPoints[uavTours[i][j]], uavPoints[uavTours[i][j+1]], 'UAV')
					) for j in range(len(uavTours[i]) - 1)
				], start=self.baseCost('UAV')),
			'UAV')
			logRiskLimit -= ugvLogSuccessChance + uavLogSuccessChance
		riskLimit = 1 - safeExp(logRiskLimit)
		self.logSuccessLimit = logRiskLimit
		self.params['FAILURE_RISK'] = riskLimit

		# solve subset of path
		super().solve(replanPoints, startPoint=startPoint, endPoint=endPoint)

		# apply changes to plan
		newUavOrder = self.solution['uav_tours']
		newUavPoints = self.solution['uav_points']
		newUgvOrder = self.solution['ugv_path']
		newUgvPoints = self.solution['ugv_point_map']

		# it is possible for the planner to duplicate a uav point if it is both the start and end of the TSP solution
		uniqueNewUavOrder = {i for tour in newUavOrder for i in tour}
		uavPointMap = {i :
			min(
				j for j in range(len(uavPoints)) if
				euclidean(newUavPoints[i], uavPoints[j]) < self.distTol
			)
			for i in uniqueNewUavOrder
		}
		reindexedNewUavOrder = [[uavPointMap[i] for i in tour] for tour in newUavOrder]
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
		for i in range(1, len(newUgvOrder) - 1): # during replan, ignoring start and end because they are repeats
			updatedUgvPoints.append(newUgvPoints[newUgvOrder[i]])
		for i in range(len(ugvOrder) - 1 - 2*numUnchangedTours, len(ugvOrder)): # after replan
			updatedUgvPoints.append(ugvPoints[ugvOrder[i]])
		updatedUgvOrder = list(range(len(updatedUgvPoints)))

		# return
		self.solveTime = self.timeInfo['TOTAL_TIME']
		return updatedUavTours, updatedUgvOrder, updatedUgvPoints
