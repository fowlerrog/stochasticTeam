# python imports
import matplotlib.pyplot as plt
from scipy.spatial.distance import euclidean
# from numpy import argmin
import random

# project imports
from pathPlanning.NodeUtils import generatePoints, createDistanceMatrix
from pathPlanning.TspUtils import solveTspWithFixedStartEnd
# from pathPlanning.EnvUtils import envFromParams
# from pathPlanning.PlotUtils import plotPoints
from pathPlanning.ClusterSolver import ClusterSolver
from pathPlanning.RunnerUtils import sigFigs

def plotPaths(TSPs, startPoints, endPoints):
	fig, ax = plt.subplots(figsize=(4, 4))
	tourColors = plt.get_cmap('tab20', len(TSPs))

	for i in range(len(TSPs)):
		# Extract x,y (uses first two coordinates if points are 3D)
		xs = [p[0] for p in TSPs[i]]
		ys = [p[1] for p in TSPs[i]]

		ax.plot(xs, ys, color=tourColors(i), marker='o', markersize=8)
		for j in range(len(TSPs[i]) - 1):
			ax.annotate("", xy=(xs[j+1], ys[j+1]), xytext=(xs[j], ys[j]),
						arrowprops=dict(arrowstyle="->", color=tourColors(i), lw=2))

		ax.plot(startPoints[i][0], startPoints[i][1], '^', color=tourColors(i), markersize=16)
		ax.plot(endPoints[i][0], endPoints[i][1], 'v', color=tourColors(i), markersize=16)

	ax.axis('equal')
	ax.grid(True)

	plt.tight_layout()
	plt.show()

# implements basic P_uav partitioning
#	by greedily adding points to one of several TSPs
#	to minimize max(dist) over all TSPs
if __name__ == '__main__':
	numPoints = 100
	numTeams = 4
	size = 4000
	seed = 2
	nSigFigs = 2
	# heuristic = 'FARTHEST'
	heuristic = 'GREEDY'
	# strategy = 'PATH_CHEAPEST_ARC'
	strategy = 'CHRISTOFIDES'
	# startPoints = [ [size//2,size//2,0] for _ in range(numTeams) ]
	# endPoints = [ [size//2, size//2, 0] for _ in range(numTeams) ]
	startPoints = [[0.,0.,0.] for _ in range(numTeams)]
	endPoints = [[4000.,4000.,0.] for _ in range(numTeams)]

	# generate points
	random.seed(seed)
	points = generatePoints( {
        'TYPE': 'Uniform',
		'NUM_POINTS': numPoints,
		'SPACE_SIZE': size,
		'FIXED_Z': 500.0,
	}, verbose=False )
	# distMatrix = createDistanceMatrix(points)

	# # initialize each TSP's length
	# tspLengths = [euclidean(s,e) for s,e in zip(startPoints, endPoints)]
	# pointGroups = [[s,e] for s,e in zip(startPoints, endPoints)]

	def totalLength(pointList):
		return sum(euclidean(pointList[i], pointList[i+1]) for i in range(len(pointList) - 1))

	# # add each point
	# for i in range(len(points)):
	# 	# try to add to each list
	# 	newLengths = []
	# 	newPoints = []
	# 	newPointsReordered = []
	# 	for j in range(numTeams):
	# 		newPoints.append(pointGroups[j][:-1] + [points[i]] + [pointGroups[j][-1]])
	# 		costMatrix = createDistanceMatrix(newPoints[j])
	# 		newTSP = solveTspWithFixedStartEnd(costMatrix, 0, len(newPoints[j]) - 1)
	# 		newPointsReordered.append([newPoints[j][idx] for idx in newTSP])
	# 		newLengths.append(totalLength(newPointsReordered[j]))
	# 	newCombinedLengths = [max(tspLengths[:j] + [newLengths[j]] + tspLengths[j+1:]) for j in range(numTeams)]
	# 	addToIdx = argmin(newCombinedLengths)
	# 	pointGroups[addToIdx] = newPointsReordered[addToIdx]
	# 	tspLengths[addToIdx] = newLengths[addToIdx]

	# print('Final paths')
	# for i in range(numTeams):
	# 	print(pointGroups[i])

	clusterSolver = ClusterSolver({'HEURISTIC':heuristic, 'STRATEGY':strategy})
	pointGroups = clusterSolver.solveClusterProblem(points, startPoints, endPoints)

	for i in range(len(pointGroups)):
		newOrdering = solveTspWithFixedStartEnd(createDistanceMatrix(pointGroups[i]), 0, len(pointGroups[i])-1, strategy=strategy)
		pointGroups[i] = [pointGroups[i][j] for j in newOrdering]
	tspLengths = [totalLength(c) for c in pointGroups]

	print('Final lengths')
	print(' '.join(str(sigFigs(t, nSigFigs)) for t in tspLengths))

	plotPaths(pointGroups, startPoints, endPoints)

	# print('Final paths')
	# for i in range(numTeams):
	# 	print(pointGroups[i])
