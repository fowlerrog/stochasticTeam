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
	@pytest.mark.parametrize("listLength", [5, 20])
	@pytest.mark.parametrize("numSegments", [3, 5])
	@pytest.mark.parametrize("costLimit", [100, 500, 1500])
	def test_partition(self, randomSeed, listLength, numSegments, costLimit):
		"""
		Tests whether the Dynamic Programming solver O(k*n^2) time
		returns the same as an exhaustive search O(n^m) time
		"""

		random.seed(randomSeed)
		costs = [Cost(*[random.randint(50, 150) for _ in range(2)]) for _ in range(listLength)]
		func = lambda c: self.logSuccessChance(c, costLimit)

		# solve with dp
		partitionSolver = PartitionSolver(costs, func, False)
		dpBestCuts, dpLogSuccess, dpSegmentLogSuccesses = partitionSolver.solvePartitionProblem(numSegments)

		# solve exhaustively
		exBestCuts, exLogSuccess, exSegmentLogSuccesses = solvePartitionExhaustively(costs, numSegments, func)

		# check for consistency
		assert dpBestCuts == exBestCuts
		assert abs(dpLogSuccess - exLogSuccess) < 1e-1
		assert all(abs(dpSegmentLogSuccesses[i] - exSegmentLogSuccesses[i]) < 1e-1 for i in range(numSegments))

	@pytest.mark.parametrize("randomSeed", range(5))
	@pytest.mark.parametrize("listLength", [5, 20])
	@pytest.mark.parametrize("numSegments", [3, 5])
	# @pytest.mark.parametrize("costLimit", [100, 500, 1500])
	def test_partitionConcave(self, randomSeed, listLength, numSegments):#, costLimit):
		"""
		Tests whether the Dynamic Programming solver O(k*n^2) time
		returns the same in concave mode for a concave in O(k*n*logn) time

		We can also show here that our f is not concave, because the test fails for some cases if we use our f instead of the concave function
		"""

		random.seed(randomSeed)
		costs = [Cost(*[random.randint(50, 150) for _ in range(2)]) for _ in range(listLength)]
		# func = lambda c: self.logSuccessChance(c, costLimit)
		func = lambda c: -(c.value[0] / 100)**2

		# solve with dp
		partitionSolver = PartitionSolver(costs, func, False)
		dpBestCuts, dpLogSuccess, dpSegmentLogSuccesses = partitionSolver.solvePartitionProblem(numSegments)

		# solve with dp concave
		partitionSolver2 = PartitionSolver(costs, func, True)
		dpBestCuts2, dpLogSuccess2, dpSegmentLogSuccesses2 = partitionSolver2.solvePartitionProblem(numSegments)

		# check for consistency
		assert dpBestCuts == dpBestCuts2
		assert abs(dpLogSuccess - dpLogSuccess2) < 1e-1
		assert all(abs(dpSegmentLogSuccesses[i] - dpSegmentLogSuccesses2[i]) < 1e-1 for i in range(numSegments))
