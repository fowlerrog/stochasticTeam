# python imports
from scipy.stats import norm
import math
import pytest
import random

# project imports
from pathPlanning.OurPlanner import Cost
from pathPlanning.PartitionSolver import PartitionSolver, solvePartitionExhaustively

class TestPartition:

	def logSuccessChance(self, cost, costLimit):
		"""f(s) from ourPlanner; it is important that this is shaped like the real function"""
		INF = float('inf')
		mean = cost.value[0]
		var = cost.value[1]
		limit = costLimit
		return norm.logsf( (-limit + mean) / math.sqrt(var) ) if var > 0 \
			else 0 if mean < limit \
			else -INF

	@pytest.mark.parametrize("randomSeed", range(5))
	@pytest.mark.parametrize("listLength", [20])
	@pytest.mark.parametrize("numSegments", [3, 5])
	@pytest.mark.parametrize("costLimit", [100, 300, 500])
	def test_partition(self, randomSeed, listLength, numSegments, costLimit):
		"""
		Tests whether the Dynamic Programming solver O(k*n^2) time
		returns the same as an exhaustive search O(n^m) time
		"""

		random.seed(randomSeed)
		costs = [Cost(*[random.randint(50, 150) for _ in range(2)]) for _ in range(listLength)]

		# solve with dp
		partitionSolver = PartitionSolver(costs, lambda c: self.logSuccessChance(c, costLimit), False)
		dpBestCuts, dpLogSuccess, dpSegmentLogSuccesses = partitionSolver.solvePartitionProblem(numSegments)

		# solve exhaustively
		exBestCuts, exLogSuccess, exSegmentLogSuccesses = solvePartitionExhaustively(costs, numSegments, lambda c: self.logSuccessChance(c, costLimit))

		# check for consistency
		assert dpBestCuts == exBestCuts
		assert abs(dpLogSuccess - exLogSuccess) < 1e-1
		assert all(abs(dpSegmentLogSuccesses[i] - exSegmentLogSuccesses[i]) < 1e-1 for i in range(numSegments))
