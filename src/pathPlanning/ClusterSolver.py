# python imports
import numpy as np
from random import choice
import time

# project imports
from .EnvUtils import envFromParams
from .NodeUtils import createDistanceMatrix, createFunctionMatrix, createSubmatrix
from .TspUtils import solveTspWithFixedStartEnd
from .RunnerUtils import sigFigs

class ClusterSolver(object):
	"""
	Solver class that divides a number of points between teams
	where each team has a given start and end point
	in order to minimize the maximum path distance across teams
	"""

	params = {} # input parameters
	solution = {} # results of solving
	timeInfo = {} # solution timing information
	env = None # environment model, if necessary

	def __init__(self, params):
		"""
		Constructor:
		params = dict of parameters
		"""
		self.params = params

		if 'ENVIRONMENT' in params:
			envParams = params['ENVIRONMENT']
			self.env = envFromParams(envParams)

	def solveClusterProblem(self, points, starts, ends):
		"""
		distributes list of points between clusters
		"""
		assert len(starts) == len(ends), 'Must have same number of team start and end points'
		print(f'Distributing {len(points)} points between {len(starts)} teams')

		startTime = time.perf_counter()

		if len(points[0]) > len(starts[0]): # trim or extend dimensions
			starts = [[*s, *[0 * len(points[0]) - len(s)]] for s in starts]
			ends = [[*e, *[0 * len(points[0]) - len(e)]] for e in ends]
		elif len(points[0]) < len(starts[0]):
			starts = [s[:len(points[0])] for s in starts]
			ends = [e[:len(points[0])] for e in ends]

		if 'HEURISTIC' in self.params and self.params['HEURISTIC'] == 'GREEDY':
			clusterPoints, tspLengths = self.solveClusterProblemGreedy(points, starts, ends)
		else:
			clusterPoints, tspLengths = self.solveClusterProblemNonGreedy(points, starts, ends)

		# save solution and time info
		endTime = time.perf_counter()
		self.timeInfo = {
			"CLUSTER_TIME": endTime - startTime
		}
		self.solution = {
			"clusters": clusterPoints,
			"tsp_lengths": tspLengths
		}
		print("\n".join([f'\tTeam {i} : {len(clusterPoints[i])} points : {sigFigs(tspLengths[i], 2)} path length' for i in range(len(clusterPoints))]))

		return clusterPoints

	def solveClusterProblemGreedy(self, points, starts, ends):
		masterPoints = points + starts + ends
		costMatrix = self.createCostMatrix(masterPoints, agentType='UAV')
		tspStrategy = self.params['STRATEGY'] if 'STRATEGY' in self.params else None

		# initialize each TSP's length
		numPoints = len(points)
		numTeams = len(starts)
		clusters = [[numPoints + i, numPoints + numTeams + i] for i in range(numTeams)]
		tspLengths = [self.totalLength(costMatrix, p) for p in clusters]
		remainingPoints = list(range(numPoints))

		# initialize choice matrix: newLength[point, team]
		newLengths = np.zeros((numPoints, numTeams))
		for i in range(numPoints):
			for j in range(numTeams):
				newLengths[i,j] = self.solveTSPlength(costMatrix, i, clusters[j], tspStrategy)

		# add each point
		while len(remainingPoints) > 0:
			# evaluate makespans
			makespans = np.zeros((len(remainingPoints), numTeams))
			for i,p in enumerate(remainingPoints):
				for j in range(numTeams):
					makespans[i,j] = max(*tspLengths[:j], *tspLengths[j+1:], newLengths[p,j])

			# choose cheapest insertion
			minPointInd, minTeam = np.unravel_index(np.argmin(makespans, axis=None), makespans.shape)
			minPoint = remainingPoints[minPointInd]

			# add to cluster
			clusters[minTeam] = clusters[minTeam][:-1] + [minPoint, clusters[minTeam][-1]] # keep it in order
			tspLengths[minTeam] = newLengths[minPoint, minTeam]
			remainingPoints.pop(minPointInd)

			# update remaining newLength values
			for p in remainingPoints:
				newLengths[p,minTeam] = self.solveTSPlength(costMatrix, p, clusters[minTeam], tspStrategy)

		# convert from indices to points
		clusterPoints = [ 
			[	points[i] if i < numPoints else
				starts[i - numPoints] if i < numPoints + numTeams else
				ends[i - numPoints - numTeams]
			for i in c ]
		for c in clusters ]

		return clusterPoints, tspLengths

	def solveClusterProblemNonGreedy(self, points, starts, ends):
		masterPoints = points + starts + ends
		costMatrix = self.createCostMatrix(masterPoints, agentType='UAV')
		tspStrategy = self.params['STRATEGY'] if 'STRATEGY' in self.params else None

		# initialize each TSP's length
		numPoints = len(points)
		numTeams = len(starts)
		clusters = [[numPoints + i, numPoints + numTeams + i] for i in range(numTeams)]
		tspLengths = [self.totalLength(costMatrix, p) for p in clusters]
		remainingPoints = list(range(numPoints))

		# add each point
		while len(remainingPoints) > 0:
			# choose a point
			pIndex = self.choosePoint(remainingPoints, clusters, costMatrix)

			# test each cluster
			tempTspLengths = []
			for c in clusters:
				tempTspLengths.append(self.solveTSPlength(costMatrix, pIndex, c, tspStrategy))

			# choose a cluster
			muStar = np.argmin([
				np.max([
					tspLengths[:i] + [tempTspLengths[i]] + tspLengths[i+1:] # replace one team's length with the new length
				]) for i in range(numTeams) # then take max over all teams
			]) # then take minimum option
			muStarTspLength = tempTspLengths[muStar]

		# add to cluster
		clusters[muStar] = clusters[muStar][:-1] + [pIndex, clusters[muStar][-1]] # keep it in order
		tspLengths[muStar] = muStarTspLength
		remainingPoints.remove(pIndex)

		# convert from indices to points
		clusterPoints = [ 
			[	points[i] if i < numPoints else
				starts[i - numPoints] if i < numPoints + numTeams else
				ends[i - numPoints - numTeams]
			for i in c ]
		for c in clusters ]

		return clusterPoints, tspLengths

	def choosePoint(self, points, clusters, costMatrix):
		"""Chooses a point from points"""
		if 'HEURISTIC' in self.params:
			if self.params['HEURISTIC'] == 'FARTHEST': # choose largest distance to closest point in clusters
				return points[np.argmax([np.min([costMatrix[p,b] for c in clusters for b in c]) for p in points])]
			elif self.params['HEURISTIC'] == 'CLOSEST': # choose smallest distance to closest point in clusters
				return points[np.argmin([np.min([costMatrix[p,b] for c in clusters for b in c]) for p in points])]
			if self.params['HEURISTIC'] == 'RANDOM':
				return choice(points)
		return points[0] # fallback

	# TSP solver helper
	def solveTSPlength(self, costMatrix, thisPIndex, thisCluster, tspStrategy):
		costSubMatrix = createSubmatrix(costMatrix, thisCluster[:-1] + [thisPIndex, thisCluster[-1]])
		tspSolution = solveTspWithFixedStartEnd(costSubMatrix, 0, len(thisCluster), strategy=tspStrategy)
		return self.totalLength(costSubMatrix, tspSolution)

	def createCostMatrix(self, points, agentType = ''):
		"""Fills a cost matrix for list of tuples"""
		if self.env == None:
			print('No planning environment, defaulting to distance for TSP')
			return createDistanceMatrix(points)
		return createFunctionMatrix(points,
			lambda point1, point2 : self.env.estimateMean(point1, point2, agentType)
		)

	def totalLength(self, costMatrix, pointList):
		return sum(costMatrix[pointList[i]][pointList[i+1]] for i in range(len(pointList) - 1))
