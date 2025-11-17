
# python imports
import math

def solvePartitionProblem(costs, numSegments, f):
	"""
	Solves the optimal partition problem, where a list is cut into pieces such that
	f(costs[ : cuts[0]]) + f(costs[cuts[0] : cuts[1]]) + ... + f(costs[cuts[numCuts-1] : ])
	is maximized

	Returns:
		maxSum = sum(f)
		cuts = cut indices
	"""

	numElements = len(costs)
	assert 1 <= numSegments <= numElements, "Cannot cut %d elements into %d segments"%(numElements, numSegments)

	# precompute running sums
	runningTotalCosts = [costs[0] * 0] * (numElements+1)
	for i in range(1, numElements+1):
		runningTotalCosts[i] = runningTotalCosts[i-1] + costs[i-1]

	def segmentCost(a, b):
		return runningTotalCosts[b] - runningTotalCosts[a]

	# create dynamic programming matrix	
	# DP[i][j] = max sum for first i items in j segments
	INF = float('inf')
	DP = [[-INF] * (numSegments + 1) for _ in range(numElements + 1)]
	DP[0][0] = 0 # base case

	# for backtracking later
	parents = [[None] * (numSegments + 1) for _ in range(numElements + 1)]

	for thisSegment in range(1, numSegments + 1):	# number of segments
		for thisCut in range(1, numElements + 1):	# end of last segment
			bestSum = -INF
			bestParent = None
			# start point for segment before this one
			for prevCut in range(thisSegment - 1, thisCut):
				thisSum = DP[prevCut][thisSegment-1] + f(segmentCost(prevCut, thisCut))
				if thisSum > bestSum:
					bestSum = thisSum
					bestParent = prevCut
			DP[thisCut][thisSegment] = bestSum
			parents[thisCut][thisSegment] = bestParent

	# backtrack to get each cut location
	cuts = []
	thisCut = parents[numElements][numSegments]
	segmentValues = [f(segmentCost(thisCut, numElements))]
	thisSegment = numSegments - 1
	while thisSegment > 0:
		parent = parents[thisCut][thisSegment]
		cuts.append(thisCut)
		segmentValues = [f(segmentCost(parent, thisCut))] + segmentValues
		thisCut = parent
		thisSegment -= 1
	cuts.reverse() # decreasing -> increasing

	return cuts, DP[numElements][numSegments], segmentValues

def safeExp(f):
	"""Returns a safe exponential that will not overflow / underflow"""
	try:
		return math.exp(f)
	except OverflowError:
		return float('inf') if f > 0 else 0.0
