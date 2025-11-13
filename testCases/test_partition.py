# python imports
from scipy.stats import norm
import math
import pytest
import random
from functools import lru_cache

# project imports
from pathPlanning.OurPlanner import Cost
from pathPlanning.PartitionSolver import solvePartitionProblem

class TestPartition:

	@pytest.mark.parametrize("randomSeed", range(5))
	@pytest.mark.parametrize("listLength", [20])
	@pytest.mark.parametrize("numSegments", [3, 5])
	@pytest.mark.parametrize("costLimit", [100, 300, 500])
	def test_partition(self, randomSeed, listLength, numSegments, costLimit):
		"""
		Tests whether the Dynamic Programming solver O(k*n^2) time
		returns the same as an exhaustive search O(n^m) time
		"""

		# use f(s) from ourPlanner; it is important that this is shaped like the real function
		INF = float('inf')
		def logSuccessChance(cost):
			mean = cost.value[0]
			var = cost.value[1]
			limit = costLimit
			return norm.logcdf( (limit - mean) / math.sqrt(var) ) if var > 0 \
				else INF if mean < limit \
				else -INF

		random.seed(randomSeed)
		costs = [Cost(*[random.randint(50, 150) for _ in range(2)]) for _ in range(listLength)]

		# solve with dp
		dpLogSuccess, dpBestCuts = solvePartitionProblem(costs, numSegments, logSuccessChance)

		# solve exhaustively
		exCuts = [[]]
		for cutNum in range(numSegments - 1):
			exCuts = [
				c + [i]				# append another cut to this set of cuts
				for c in exCuts		# for all current sets of cuts
				for i in range(c[-1] + 1 if len(c) > 0 else 1, listLength - numSegments + cutNum + 2) # for all possible cuts, [1,n) > c[-1]
			]

		@lru_cache(maxsize=None)
		def segmentLogSuccess(a, b):
			return logSuccessChance(sum(costs[a:b], start=Cost(0,0)))

		segmentLogSuccesses = [
			segmentLogSuccess(0,c[0]) +
			sum(segmentLogSuccess(c[i],c[i+1]) for i in range(len(c)-1)) +
			segmentLogSuccess(c[-1],len(costs))
			for c in exCuts
			]
		labeledCosts = zip(segmentLogSuccesses, exCuts)
		e = max(labeledCosts)
		exLogSuccess = e[0]
		exBestCuts = e[1]

		def safeExp(f):
			# exponential may overflow / underflow
			try:
				return math.exp(f)
			except OverflowError:
				return float('inf') if f > 0 else 0.0

		dpSuccess = safeExp(dpLogSuccess)
		exSuccess = safeExp(exLogSuccess)

		print('dplogsuccess', dpLogSuccess, 'dpsuccess', dpSuccess, 'dpcuts', dpBestCuts)
		print('exlogsuccess', exLogSuccess, 'exsuccess', exSuccess, 'excuts', exBestCuts)
		assert dpBestCuts == exBestCuts
