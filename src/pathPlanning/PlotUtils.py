# python imports
import os
import matplotlib.pyplot as plt
import numpy as np

# project imports
from .RunnerUtils import loadPlanResultsFromFolder

def plotPoints(points, filename=None, show=True):
	"""Plot points with no path"""
	plt.figure()
	xs = [p[0] for p in points]
	ys = [p[1] for p in points]
	plt.scatter(xs, ys, marker='o')
	plt.title('Reordered Points (no tour found)')
	plt.axis('equal')
	plt.grid(True)

	if filename is not None:
		plt.savefig(filename)
	if show:
		plt.show()

def plotPath(points, filename=None, show=True):
	"""Plot points with path"""
	# Extract x,y (uses first two coordinates if points are 3D)
	xs = [p[0] for p in points]
	ys = [p[1] for p in points]

	# Plot the tour path with points
	plt.plot(xs, ys, marker='o')

	# Mark start and end (first and last in the tour order)
	plt.scatter(xs[0], ys[0], marker='s', s=100, label='Start (closest)')
	plt.scatter(xs[-1], ys[-1], marker='X', s=120, label='End (closest)')

	plt.title('TSP Tour with Fixed Start/End (reordered points)')
	plt.axis('equal')
	plt.grid(True)
	plt.legend()

	if filename is not None:
		plt.savefig(filename)
	if show:
		plt.show()

def plotCycles(uavCycles, uavPoints, ugvPointMap, ugvPath, filename=None, show=True):
	"""Plot points, clustered into cycles"""
	cycleColorGroups = [0] * len(uavPoints)
	for cix, cycle in enumerate(uavCycles):
		for p in cycle:
			cycleColorGroups[p] = cix

	cycleColors = plt.cm.get_cmap('tab20', len(uavCycles))
	fig, ax = plt.subplots(figsize=(8, 6))
	# print(f"Mapping to points: {mapping_to_points}")
	uavPoints = np.array(uavPoints)[:, :2]

	# plot colored circles for uav points
	for i, point in enumerate(uavPoints):
		x,y = point
		# print(x, y, cycleColorGroups[i], cycleColors(cycleColorGroups[i]))
		color = cycleColors(cycleColorGroups[i])
		ax.plot(x, y, 'o', color=color, markersize=16)
		ax.text(x, y, f"{i}a", fontsize=16, color=color, verticalalignment='top')

	# plot colored circles for ugv path (including start, end)
	i = 0
	while i < len(ugvPath) - 1:
		x,y = ugvPointMap[ugvPath[i]]
		color = 'g' if i == 0 else 'r' if i == len(ugvPath) - 2 else 'k'
		ax.plot(x, y, 'x', color=color, markersize=16)
		labelNum = str(i)
		while ugvPointMap[ugvPath[i]] == ugvPointMap[ugvPath[i+1]]:
			i += 1
			labelNum += ',' + str(i)
		ax.text(x, y, labelNum + 'g', fontsize=16, color=color, verticalalignment='bottom')
		i += 1

	# plot uav path
	for cycle in uavCycles:
		for i in range(len(cycle) - 1):
			x1, y1 = uavPoints[cycle[i]]
			x2, y2 = uavPoints[cycle[i + 1]]
			ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
						arrowprops=dict(arrowstyle="-", color='black', lw=2))

	# plot ugv path
	for i in range(len(ugvPath)):
		x1, y1 = ugvPointMap[ugvPath[i]]
		x2, y2 = ugvPointMap[ugvPath[(i + 1) % len(ugvPath)]]
		if x1 == x2 and y1 == y2:
			continue
		print(f"Drawing edge from {ugvPath[i]} to {ugvPath[(i + 1) % len(ugvPath)]}: ({x1}, {y1}) to ({x2}, {y2})")
		ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
					arrowprops=dict(arrowstyle="->", color='red', lw=2))

	# ax.set_xlim(-100, 4000)
	# ax.set_ylim(-100, 4000)
	ax.grid(True)
	ax.relim()
	ax.autoscale_view()
	ax.set_title("GTSP with Release & Collect (Full Tour incl. Dummy)")
	ax.set_aspect('equal')

	if filename is not None:
		plt.savefig(filename)
	if show:
		plt.show()

def plotOriginalCycles(originalPoints, originalCycles, startPoint=None, endPoint=None, filename=None, show=True):
	colors = plt.cm.get_cmap('tab20', len(originalCycles))
	fig, ax = plt.subplots(figsize=(8, 6))
	for i, cycle in enumerate(originalCycles):
		color = colors(i)
		for ptIdx in cycle:
			x, y, _ = originalPoints[ptIdx]
			ax.plot(x, y, 'o', color=color)
			ax.text(x + 10, y + 10, f"{ptIdx}", fontsize=8, color=color)
		cycleCoords = [originalPoints[ptIdx][:2] for ptIdx in cycle]
		cycleCoords.append(cycleCoords[0])  # close the cycle
		for j in range(len(cycleCoords) - 1):
			x1, y1 = cycleCoords[j]
			x2, y2 = cycleCoords[j + 1]
			ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
						arrowprops=dict(arrowstyle="->", color=color, lw=1.5))
	if startPoint:
		ax.plot(startPoint[0], startPoint[1], 's', color='black', markersize=8, label='Start')
		ax.text(startPoint[0] + 10, startPoint[1] + 10, 'Start', fontsize=9, color='black')
	if endPoint:
		ax.plot(endPoint[0], endPoint[1], 'D', color='black', markersize=8, label='End')
		ax.text(endPoint[0] + 10, endPoint[1] + 10, 'End', fontsize=9, color='black')

	releasePoints = [originalPoints[cycle[0]] for cycle in originalCycles if len(cycle) > 0]
	for i in range(len(releasePoints) - 1):
		x1, y1 = releasePoints[i][:2]
		x2, y2 = releasePoints[i + 1][:2]
		ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
					arrowprops=dict(arrowstyle="->", color='gray', lw=2, ls='--'))

	ax.set_title("Original UAV Cycles + UGV Release Path")
	ax.set_aspect('equal')
	ax.grid(True)

	if filename is not None:
		plt.savefig(filename)
	if show:
		plt.show()

def plotPlanFromFolder(folderPath):
	"""Generates plots for a TSP path from a results folder"""
	absFolderPath = os.path.abspath(folderPath)

	plan = loadPlanResultsFromFolder(absFolderPath)
	if plan is None:
		return

	TSPFigureName = os.path.join(absFolderPath, 'TSP_path.png')
	if 'uav_points' in plan:
		plotPath(plan['uav_points'], filename=TSPFigureName, show=False)
	# else:
	# 	# If no solution, just scatter the reordered points for reference
	# 	plotPoints(points,
	# 				filename=TSPFigureName)

	cyclesFigureName = os.path.join(absFolderPath, 'uav_cycles.png')
	plotCycles(plan['uav_cycles'],
			    plan['uav_points'],
				plan["ugv_point_map"],
				plan["ugv_path"],
				cyclesFigureName,
				False)

	plt.show()
