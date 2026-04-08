# python imports
import pytest
import random
from scipy.spatial.distance import euclidean

# project imports
from pathPlanning.ClusterSolver import ClusterSolver
from pathPlanning.NodeUtils import createDistanceMatrix, generatePoints

class TestCluster:

	@pytest.mark.parametrize("heuristic", ['FARTHEST', 'CLOSEST'])
	@pytest.mark.parametrize("invert", [True, False])
	def test_pointChoice(self, heuristic, invert):
		"""
		Tests whether the cluster solver can choose points based on a heuristic
		"""

		# generate points and existing clusters
		points = [(0, 10, 10), (0, 20, 10)]
		clusters = [[(0, 0, 0), (20, 0, 0)]]
		costMatrix = createDistanceMatrix(points + sum(clusters, start=[]))

		# create clusterer
		clusterSolver = ClusterSolver({'HEURISTIC':heuristic})
		pointIndices = list(range(len(points))) # point indices = [0, 1]
		clusterIndices = [[r + len(points) + sum(len(clusters[j-1]) for j in range(i)) for r in range(len(clusters[i]))] for i in range(len(clusters))] # cluster indices = [2, 3]
		pIndex = clusterSolver.choosePoint(
			pointIndices[::-1] if invert else pointIndices, # should still work if we give the point index list out of order
			clusterIndices,
			costMatrix
		)

		# check for consistency
		if heuristic == 'FARTHEST':
			assert pIndex == 1
		elif heuristic == 'CLOSEST':
			assert pIndex == 0

	@pytest.mark.parametrize("randomSeed", range(5))
	@pytest.mark.parametrize("numPoints", [20])
	@pytest.mark.parametrize("numTeams", [3, 5])
	@pytest.mark.parametrize("heuristic", [None, 'FARTHEST', 'CLOSEST', 'RANDOM'])
	def test_completeness(self, randomSeed, numPoints, numTeams, heuristic):
		"""
		Tests whether the cluster solver places each point in exactly one cluster
		"""

		spaceSize = 100
		z = 10
		eps = 1e-6

		# generate points and start/end points
		random.seed(randomSeed)
		points = generatePoints( {
			'TYPE': 'Uniform',
			'NUM_POINTS': numPoints,
			'SPACE_SIZE': spaceSize,
			'FIXED_Z': z,
		}, verbose=False )
		starts = generatePoints( {
			'TYPE': 'Uniform',
			'NUM_POINTS': numTeams,
			'SPACE_SIZE': spaceSize,
			'FIXED_Z': 0.0,
		}, verbose=False )
		ends = generatePoints( {
			'TYPE': 'Uniform',
			'NUM_POINTS': numTeams,
			'SPACE_SIZE': spaceSize,
			'FIXED_Z': 0.0,
		}, verbose=False )

		# create solver and solve
		clusterSolver = ClusterSolver({'HEURISTIC':heuristic})
		clusters = clusterSolver.solveClusterProblem(points, starts, ends)

		# check that each ground point is first or last, and the rest are air points
		assert all(abs(c[0][-1]) < eps and abs(c[-1][-1]) < eps for c in clusters)
		assert all(all(abs(c[i][-1] - z) < eps for i in range(1,len(c)-1)) for c in clusters)

		# check that each air point is added once
		airPoints = sum([c[1:-1] for c in clusters], start=[])
		for p in points:
			found = False
			for i in range(len(airPoints)):
				if euclidean(p, airPoints[i]) < eps:
					found = True
					airPoints.pop(i)
					break
			assert found
		assert len(airPoints) == 0
