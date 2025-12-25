
# python imports
from collections.abc import Callable, Iterable
import numpy as np
from itertools import repeat
from functools import lru_cache

class PartitionSolver():
	"""
	Solver class that solves the optimal partition problem,
	where a list is cut into pieces such that
	f(costs[ : cuts[0]]) + f(costs[cuts[0] : cuts[1]]) + ... + f(costs[cuts[numCuts-1] : ])
	is maximized
	"""

	costs: Iterable		# cost list of length n
	f: Callable		# function

	DP: np.array	# dynamic programming matrix: DP[j-1][i-1] = max sum for first i items in j segments
	parents: np.array	# for backtracking: parents[j-1][i-1] = argmax in p of DP[j-2][p-1] + f(costs[p:i])
	currentNumSegments: int # number of segments for which DP[numSegments-1][n-1] has been evaluated
	fCache: dict	# function cache {(a,b) : f(a,b)}

	concave: bool	# whether parents[j][i] is nondecreasing with i, which allows us to solve for a given j in O(n log(n)) instead of O(n^2)
	# parallel: bool	# whether to use multiprocessing to parallelize things (which is only faster for a very large problem or a slow f)
					# parallel is set automatically by passing a Manager and Pool to solvePartitionProblem()
					# note that concave and parallel are mutually exclusive

	runningTotalCosts: list	# running sums of costs

	def __init__(self, costs, f,
			concave = False,
		):
		self.costs = costs
		self.f = f

		self.DP = np.zeros( (0, len(costs)) )
		self.parents = np.zeros( (0, len(costs)), dtype=np.int8 )
		self.currentNumSegments = 0
		self.fCache = {}

		self.concave = concave

		# precompute running sums (prepended with 0 because of fencepost problem)
		self.runningTotalCosts = [costs[0] * 0] * (len(costs) + 1)
		for i in range(1, len(costs) + 1):
			self.runningTotalCosts[i] = self.runningTotalCosts[i-1] + costs[i-1]

	def evalFunction(self, a, b, runningTotalCosts, *args, **kwargs):
		"""Helper function to speed up function evaluation"""
		value = self.fCache.get( (a,b), None)
		if value is None:
			value = self.f(runningTotalCosts[b] - runningTotalCosts[a], *args, **kwargs)
			self.fCache[ (a,b) ] = value
		return value

	def solvePartitionProblem(self, numSegments, manager = None, pool = None):
		"""
		Solves the partition problem for any unsolved number of segments
		Executes in parallel if manager = multiprocessing.Manager() and pool = multiprocessing.Pool() are provided
		Returns:
			maxSum = sum(f)
			cuts = cut indices
		"""

		numElements = len(self.costs)
		assert 1 <= numSegments <= numElements, "Cannot cut %d elements into %d segments"%(numElements, numSegments)

		while self.currentNumSegments < numSegments:
			self.solvePartitionProblemLayer(self.currentNumSegments + 1, manager=manager, pool=pool)
			self.currentNumSegments += 1

		cuts, segmentValues = self.getCuts(numElements, numSegments)
		return [c + 1 for c in cuts], self.DP[numSegments - 1][numElements - 1], segmentValues

	def solvePartitionProblemLayer(self, numSegments, manager = None, pool = None):
		"""Solves for a given j, assuming j-1 is already solved"""

		numElements = len(self.costs)

		# evaluate the [0:n) elements of the previous layer
		if numSegments == 1: # there is no previous layer
			pass
		elif numSegments == 2: # layer 1 is just f(segment)
			self.DP[numSegments - 2, :-1], self.parents[numSegments - 2, :-1] = self.solveRow(numSegments - 2, manager=manager, pool=pool)
		elif self.concave: # solve binary (not parallel, so nothing returned)
			self.solveRowConcave(numSegments - 2)
		else: # evaluate normally
			self.DP[numSegments - 2, :-1], self.parents[numSegments - 2, :-1] = self.solveRow(numSegments - 2, manager=manager, pool=pool)

		# expand matrices
		self.DP = np.concatenate( (self.DP, np.zeros((1, numElements))), axis=0 )
		self.parents = np.concatenate( (self.parents, np.zeros((1, numElements), dtype=np.int8)), axis=0 )

		# evaluate the last element of this layer
		if numSegments == 1: # layer 1 is just f(segment)
			self.DP[numSegments - 1, -1] = self.evalFunction(0, numElements, self.runningTotalCosts)
		else:
			self.DP[numSegments - 1, -1], self.parents[numSegments - 1, -1] = self.solveDPvalue(numSegments - 1, numElements - 1)

	def solveRow(self, j, manager = None, pool = None):
		"""
		Solves for a given j, for all i in [0:numElements]
		i = length of [0:i) sublist in consideration
		with multiprocessing threads for each element
		"""

		# in order to avoid pickling the Manager and Pool by calling pool.map(self.evalFunction), we receive them from outside
		parallel = False
		if not (manager is None or pool is None):
			parallel = True
			self.fCache = manager.dict(self.fCache)

		# declare new rows
		numElements = len(self.costs)
		dpValues = np.zeros( (1, numElements - 1) )
		parentValues = np.zeros( (1, numElements - 1), dtype=np.int8)

		if j == 0: # first layer is just f(segment) with no parent
			if parallel:
				dpValues[0, j : numElements - 1] = pool.starmap(
					self.evalFunction,
					zip(
						repeat(0),
						range(j + 1, numElements),
						repeat(self.runningTotalCosts)
					)
				)
			else:
				for thisElements in range(j + 1, numElements):
					dpValues[0, thisElements - 1] = self.evalFunction(0, thisElements, self.runningTotalCosts)
		else: # consult previous layer for DP solution
			if parallel:
				result = pool.starmap(
					self.solveDPvalue,
					zip(
						repeat(j),
						range(j, numElements - 1),
						repeat(None),
						repeat(None),
						repeat(self.DP[j-1,:]),
						repeat(self.runningTotalCosts)
					)
				)
				dpValues[0, j : numElements - 1] = np.array([el[0] for el in result])
				parentValues[0, j : numElements - 1] = np.array([el[1] for el in result])
			else:
				for thisElements in range(j + 1, numElements):
					dpValues[0, thisElements - 1], parentValues[0, thisElements - 1] = self.solveDPvalue(j, thisElements - 1)

		return dpValues, parentValues

	def solveRowConcave(self, j, iMin=None, iMax=None):
		"""
		Solves for a given j and range of i
		with a binary search for k, assuming k is nondecreasing with i
		where i varies on [iMin:iMax)
		Relying on the assumption that self.parents[j, iMax] is solved but that self.parents[j, iMin] may not be
		"""
		if iMin is None: iMin = j # leftmost limit of this row that would be evaluated
		if iMax is None: iMax = len(self.costs) - 1 # rightmost limit of this row
		if iMin >= iMax: # base case
			return

		# solve middle value of this row
		iMiddle = (iMax + iMin) // 2 # what we are currently solving
		kMin = self.parents[j, iMin] + 1 if iMin > j else j - 1 # our minimum parent is the left parent (exclusive)
		kMax = self.parents[j, iMax] if iMax < len(self.costs) - 1 and j > 0 else len(self.costs) - 1 # our maximum parent is the right parent (exclusive), though we wouldn't have filled that in if it was on the first row
		self.DP[j, iMiddle], self.parents[j, iMiddle] = self.solveDPvalue(j, iMiddle, kMin, kMax + 1)

		# recurse
		self.solveRowConcave(j, iMin, iMiddle)
		self.solveRowConcave(j, iMiddle + 1, iMax)

	def solveDPvalue(self, j, i, kMin=None, kMax=None, prevDpRow=None, runningTotalCosts=None):
		"""
		Solves for 1 value in DP matrix
		note that i and j are 0-indexed (DP[1][2] = first 3 elements in 2 segments)
		k (the j-1th cut) is allowed to vary on [kMin:kMax) which defaults to [j-1:i)
		"""
		if kMin is None: kMin = j-1
		if kMax is None: kMax = i
		if prevDpRow is None: prevDpRow = self.DP[j-1,:]
		if runningTotalCosts is None: runningTotalCosts = self.runningTotalCosts

		# find the cut that maximizes total value
		#	choosing the smaller choice for ties
		values = [ prevDpRow[k] + self.evalFunction(k+1, i+1, runningTotalCosts) for k in range(kMin, kMax) ]
		bestInd = np.argmax(values)

		return values[bestInd], bestInd + kMin

	def getCuts(self, numElements, numSegments):
		"""Backtrack to get each cut location"""

		cuts = []
		thisSegment = numSegments - 1
		thisCut = self.parents[thisSegment, numElements - 1]
		segmentValues = [self.evalFunction(thisCut + 1, numElements, self.runningTotalCosts)]
		while thisSegment > 0:
			cuts.append(thisCut)
			thisSegment -= 1
			parent = self.parents[thisSegment, thisCut] if thisSegment > 0 else -1 # check for first segment
			segmentValues = [self.evalFunction(parent + 1, thisCut + 1, self.runningTotalCosts)] + segmentValues
			thisCut = parent
		cuts.reverse() # decreasing -> increasing

		return cuts, segmentValues

class ExtendedPartitionSolver(PartitionSolver):
	"""
	Solves the partition problem,
	but allows f() evaluations to store data in self.fData
	which requires f to be of the form:
		value, data = f(sum(costs[a:b]), a, b)
	"""

	fData: dict

	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.fData = {}

	def evalFunction(self, a, b, runningTotalCosts):
		"""
		Returns the FIRST output of f normally
		and stores the SECOND in self.fData[(a,b)]
		"""

		value, data = super().evalFunction(a, b, runningTotalCosts, a, b)
		self.fData[(a,b)] = data
		return value

	def solveRow(self, j, manager=None, pool=None):
		"""
		Make self.fData shared in parallel case
		"""

		if not (manager is None or pool is None):
			self.fData = manager.dict(self.fData)
		return super().solveRow(j, manager, pool)

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

	bestInd = np.argmax(cutValues)
	bestCuts = exCuts[bestInd]
	totalValue = cutValues[bestInd]
	segmentValues = segmentEvaluation(bestCuts)

	return bestCuts, totalValue, segmentValues
