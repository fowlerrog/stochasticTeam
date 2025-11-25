
# python imports
from collections.abc import Callable
from functools import lru_cache

class PartitionSolver():
	"""
	Solver class that solves the optimal partition problem,
	where a list is cut into pieces such that
	f(costs[ : cuts[0]]) + f(costs[cuts[0] : cuts[1]]) + ... + f(costs[cuts[numCuts-1] : ])
	is maximized
	"""

	costs: list		# cost list of length n
	f: Callable		# function
	DP: list		# dynamic programming matrix: DP[j-1][i-1] = max sum for first i items in j segments
	parents: list	# for backtracking: parents[j-1][i-1] = argmax in p of DP[j-2][p-1] + f(costs[p:i])
	currentNumSegments: int # number of segments for which DP[numSegments-1][n-1] has been evaluated
	concave: bool	# whether parents[j][i] is nondecreasing with i, which allows us to solve for a given j in O(n) instead of O(n^2)

	runningTotalCosts: list	# running sums of costs

	def __init__(self, costs, f, concave = False):
		self.costs = costs
		self.f = f
		self.currentNumSegments = 0
		self.DP = []
		self.parents = []
		self.concave = concave

		# precompute running sums
		self.runningTotalCosts = [costs[0] * 0] * (len(costs) + 1)
		for i in range(1, len(costs) + 1):
			self.runningTotalCosts[i] = self.runningTotalCosts[i-1] + costs[i-1]

	def segmentCost(self, a, b):
		return self.runningTotalCosts[b] - self.runningTotalCosts[a]

	def solvePartitionProblem(self, numSegments):
		"""
		Returns:
			maxSum = sum(f)
			cuts = cut indices
		"""

		numElements = len(self.costs)
		assert 1 <= numSegments <= numElements, "Cannot cut %d elements into %d segments"%(numElements, numSegments)

		while self.currentNumSegments < numSegments:
			self.solvePartitionProblemLayer(self.currentNumSegments + 1)
			self.currentNumSegments += 1

		cuts, segmentValues = self.getCuts(numElements, numSegments)
		return [c + 1 for c in cuts], self.DP[numSegments - 1][numElements - 1], segmentValues

	def solvePartitionProblemLayer(self, numSegments):
		"""Solves for a given j, assuming j-1 is already solved"""

		numElements = len(self.costs)

		# expand matrices
		self.DP.append([0] * numElements)
		self.parents.append([None] * numElements)

		# evaluate the [0:n) elements of the previous layer
		if numSegments == 1: # there is no previous layer
			pass
		elif numSegments == 2: # layer 1 is just f(segment)
			for thisElements in range(numSegments - 1, numElements): # length of list in consideration
				self.DP[numSegments - 2][thisElements - 1] = self.f(self.runningTotalCosts[thisElements])
				self.parents[numSegments - 2][thisElements - 1] = -1
		elif self.concave: # solve binary
			self.solveRowConcave(numSegments - 2)
		else: # evaluate normally
			for thisElements in range(numSegments - 1, numElements): # length of list in consideration
				self.solveDPvalue(numSegments - 2, thisElements - 1)

		# evaluate the last element of this layer
		if numSegments == 1: # layer 1 is just f(segment)
			self.DP[numSegments - 1][-1] = self.f(self.runningTotalCosts[numElements])
			self.parents[numSegments - 1][-1] = -1
		else:
			self.solveDPvalue(numSegments - 1, numElements - 1)

	def solveRowConcave(self, j, iMin=None, iMax=None):
		"""
		Solves for a given j and range of i
		with a binary search for k, assuming k is nondecreasing with i
		where i varies on [iMin:iMax]
		"""
		if iMin is None: iMin = j
		if iMax is None: iMax = len(self.costs) - 1
		if iMin >= iMax: # base case
			return

		# solve middle value of this row
		iMiddle = (iMax + iMin) // 2
		kMin = self.parents[j][iMin] if iMin >= j else j
		kMax = self.parents[j][iMax] if iMax < len(self.costs) else len(self.costs) - 1
		self.solveDPvalue(j, iMiddle, kMin, kMax + 1)

		# recurse
		self.solveRowConcave(j, iMin, iMiddle)
		self.solveRowConcave(j, iMiddle + 1, iMax)

	def solveDPvalue(self, j, i, kMin=None, kMax=None):
		"""
		Solves for 1 value in DP matrix
		note that i and j are 0-indexed (DP[1][2] = first 3 elements in 2 segments)
		k (the j-1th cut) is allowed to vary on [kMin:kMax] which defaults to [j-1:i]
		"""
		if kMin is None: kMin = j-1
		if kMax is None: kMax = i
		val = max(
			( self.DP[j-1][k] + self.f(self.segmentCost(k+1, i+1)), k )
			for k in range(kMin, kMax)
			)
		self.DP[j][i] = val[0]
		self.parents[j][i] = val[1]

	def getCuts(self, numElements, numSegments):
		"""Backtrack to get each cut location"""

		cuts = []
		thisSegment = numSegments - 1
		thisCut = self.parents[thisSegment][numElements - 1]
		segmentValues = [self.f(self.segmentCost(thisCut + 1, numElements))]
		while thisSegment > 0:
			cuts.append(thisCut)
			thisSegment -= 1
			parent = self.parents[thisSegment][thisCut]
			segmentValues = [self.f(self.segmentCost(parent + 1, thisCut + 1))] + segmentValues
			thisCut = parent
		cuts.reverse() # decreasing -> increasing

		return cuts, segmentValues

def solvePartitionExhaustively(costs, numSegments, f):
	"""Solves the maximization problem for a list of costs split into a number of segments with function f"""
	listLength = len(costs)

	# generate all lists of cuts
	exCuts = [[]]
	for cutNum in range(numSegments - 1):
		exCuts = [
			c + [i]				# append another cut to this set of cuts
			for c in exCuts		# for all current sets of cuts
			for i in range(c[-1] + 1 if len(c) > 0 else 1, listLength - numSegments + cutNum + 2) # for all possible cuts, [1,n) > c[-1]
		]

	@lru_cache(maxsize=None)
	def segmentValue(a, b):
		return f(sum(costs[a:b], start=costs[0]*0))

	segmentEvaluation = lambda c : [
		segmentValue(0,c[0]),
		*[segmentValue(c[i],c[i+1]) for i in range(len(c)-1)],
		segmentValue(c[-1],len(costs))
	]
	cutValues = [sum(segmentEvaluation(c)) for c in exCuts]
	labeledCosts = zip(cutValues, exCuts)
	e = max(labeledCosts)
	totalValue = e[0]
	bestCuts = e[1]
	segmentValues = segmentEvaluation(bestCuts)

	return bestCuts, totalValue, segmentValues
