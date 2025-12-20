# python imports
import multiprocessing
import traceback
import time
import random
from itertools import repeat

# project imports
from pathPlanning.NodeUtils import generatePoints, createDistanceMatrix
from pathPlanning.TspUtils import solveTspWithFixedStartEnd

def sq(n):
	return n*n

if __name__ == '__main__':

	seed = 0
	n = 100
	m = 5

	# generate points
	print(f'Generating {n} points')
	random.seed(seed)
	points = generatePoints(n, (0,1000), (0,1000))

	# generate cost matrices
	matrix = createDistanceMatrix(points)

	# solve m TSPs in series
	tStartSeries = time.perf_counter()
	seriesResults = [solveTspWithFixedStartEnd(matrix, 0, 1) for _ in range(m)]
	tEndSeries = time.perf_counter()

	# solve m TSPs in parallel
	tStartParallel = time.perf_counter()
	try:
		p = multiprocessing.Pool()
		tPoolParallel = time.perf_counter()
		parallelResults = p.starmap(solveTspWithFixedStartEnd, zip(repeat(matrix, m), repeat(0), repeat(1)))
	except Exception:
		print(traceback.format_exc())
	tEndParallel = time.perf_counter()

	# print results
	print('Series time', tEndSeries - tStartSeries)
	print('Parallel time', tEndParallel - tStartParallel)
	print('\tPool creation time', tPoolParallel - tStartParallel)
