# python imports
import os
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
from math import log

# project imports
from .RunnerUtils import loadPlanResultsFromFolder
from .EnvUtils import envFromParamsOrFile

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

def plotTours(uavTours, uavPoints, ugvPointMap, ugvPath, filename=None, show=True):
	"""Plot points, clustered into tours"""
	tourColorGroups = [0] * len(uavPoints)
	for cix, tour in enumerate(uavTours):
		for p in tour:
			tourColorGroups[p] = cix

	tourColors = plt.cm.get_cmap('tab20', len(uavTours))
	fig, ax = plt.subplots(figsize=(8, 6))
	# print(f"Mapping to points: {mapping_to_points}")
	uavPoints = np.array(uavPoints)[:, :2]

	# plot colored circles for uav points
	for i, point in enumerate(uavPoints):
		x,y = point
		# print(x, y, tourColorGroups[i], tourColors(tourColorGroups[i]))
		color = tourColors(tourColorGroups[i])
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
	for tour in uavTours:
		for i in range(len(tour) - 1):
			x1, y1 = uavPoints[tour[i]]
			x2, y2 = uavPoints[tour[i + 1]]
			ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
						arrowprops=dict(arrowstyle="-", color='black', lw=2))

	# plot ugv path
	for i in range(len(ugvPath)-1):
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

def plotOriginalTours(originalPoints, originalTours, startPoint=None, endPoint=None, filename=None, show=True):
	colors = plt.cm.get_cmap('tab20', len(originalTours))
	fig, ax = plt.subplots(figsize=(8, 6))
	for i, tour in enumerate(originalTours):
		color = colors(i)
		for ptIdx in tour:
			x, y, _ = originalPoints[ptIdx]
			ax.plot(x, y, 'o', color=color)
			ax.text(x + 10, y + 10, f"{ptIdx}", fontsize=8, color=color)
		tourCoords = [originalPoints[ptIdx][:2] for ptIdx in tour]
		tourCoords.append(tourCoords[0])  # close the tour
		for j in range(len(tourCoords) - 1):
			x1, y1 = tourCoords[j]
			x2, y2 = tourCoords[j + 1]
			ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
						arrowprops=dict(arrowstyle="->", color=color, lw=1.5))
	if startPoint:
		ax.plot(startPoint[0], startPoint[1], 's', color='black', markersize=8, label='Start')
		ax.text(startPoint[0] + 10, startPoint[1] + 10, 'Start', fontsize=9, color='black')
	if endPoint:
		ax.plot(endPoint[0], endPoint[1], 'D', color='black', markersize=8, label='End')
		ax.text(endPoint[0] + 10, endPoint[1] + 10, 'End', fontsize=9, color='black')

	releasePoints = [originalPoints[tour[0]] for tour in originalTours if len(tour) > 0]
	for i in range(len(releasePoints) - 1):
		x1, y1 = releasePoints[i][:2]
		x2, y2 = releasePoints[i + 1][:2]
		ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
					arrowprops=dict(arrowstyle="->", color='gray', lw=2, ls='--'))

	ax.set_title("Original UAV Tours + UGV Release Path")
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

	toursFigureName = os.path.join(absFolderPath, 'uav_tours.png')
	plotTours(plan['uav_tours'],
			    plan['uav_points'],
				plan["ugv_point_map"],
				plan["ugv_path"],
				toursFigureName,
				False)

	plt.show()

def plotSelfComparison(plannedValues, empiricalValues, labelString='', varName='', log=False, fig=None, ax=None):
	"""Plots planned values against empirical values"""
	# create figure if none given
	if fig is None or ax is None:
		fig, ax = plt.subplots()

	# get data limits
	propLineMin = max(min(plannedValues), min(empiricalValues))
	propLineMax = min(max(plannedValues), max(empiricalValues))

	# decide if log
	if log:
		plotFunction = ax.loglog
	else:
		plotFunction = ax.plot

	# plot
	plotFunction(plannedValues, empiricalValues, '.k', label=labelString)
	plotFunction([propLineMin, propLineMax], [propLineMin, propLineMax], ':k')
	ax.legend()
	ax.grid(True)
	plt.xlabel('Planned ' + varName)
	plt.ylabel('Empirical ' + varName)

	return fig, ax

def plotSolveTimes(yamlContents, xKey, yKey, fig=None, ax=None, labelString='', referenceExp=None):
	"""
	Plots some results from a plan_time_results.yaml file
		xKey = single key, or list of keys to have values summed
	"""

	# create figure if none given
	if fig is None or ax is None:
		fig, ax = plt.subplots()

	# read results from dict
	if isinstance(xKey, list):
		xData = [sum(x) for x in zip(*[yamlContents[key] for key in xKey])]
	else:
		xData = yamlContents[xKey]
	if isinstance(yKey, list):
		yData = [sum(y) for y in zip(*[yamlContents[key] for key in yKey])]
	else:
		yData = yamlContents[yKey]

	# combine unique x values
	dataDict = {}
	while len(xData) > 0:
		thisX = xData[-1]
		thisY = []
		for i in range(len(xData) - 1, -1, -1):
			if xData[i] == thisX:
				xData.pop(i)
				thisY.append(yData.pop(i))
		dataDict[thisX] = {
			'values' : thisY,
			'min' : min(thisY),
			'mean' : np.mean(thisY),
			'max' : max(thisY)
		}

	# plot line for each group
	xSorted = sorted(dataDict.keys())
	for x in xSorted:
		plt.loglog(
			[x, x, x], [
				dataDict[x]['min'],
				dataDict[x]['mean'],
				dataDict[x]['max']
			], '-+b')

	# plot mean line
	plt.loglog(
		xSorted,
		[dataDict[x]['mean'] for x in xSorted],
		'-b',
		label=labelString
	)

	# plot fitted line
	logX = []
	logY = []
	for x in xSorted:
		logX.extend([log(x)] * len(dataDict[x]['values']))
		logY.extend([log(y) for y in dataDict[x]['values']])
	exp, mult = np.polyfit(logX, logY, 1)

	polyFit = lambda x : mult * x ** exp
	xLimits = [min(xSorted), max(xSorted)]
	yXMin = dataDict[xLimits[0]]['mean']
	plt.loglog(
		xLimits,
		evalNormalizedFunction(polyFit, xLimits, yXMin),
		'--r',
		label=f'O(n^{exp:.2f})'
	)

	# plot reference exponent
	if referenceExp is not None:
		plt.loglog(
			xLimits,
			evalNormalizedFunction(lambda x : x ** referenceExp, xLimits, yXMin),
			'--k',
			label=f'O(n^{referenceExp})'
		)

		# # plot the O(n^(p-1) log(n)) case
		# xRange = np.logspace(*[log(x) for x in xLimits], base=np.e)
		# plt.loglog(
		# 	xRange,
		# 	evalNormalizedFunction(lambda x : x ** (referenceExp - 1) * log(x), xRange, yXMin),
		# 	'--g',
		# 	label=f'O(n^{referenceExp-1} log(n))'
		# )

	# set x labels to integers
	ax.xaxis.set_major_locator(ticker.LogLocator(base=10, subs='all'))
	ax.xaxis.set_major_formatter(lambda x, pos : '%d'%int(x))
	ax.xaxis.set_minor_locator(ticker.NullLocator())

	return fig, ax

def evalNormalizedFunction(f, xRange, fOfXMin):
	"""Evaluates f(xRange) normalized to f(min(xRange)) = fOfXMin"""
	return [fOfXMin * f(x) / f(min(xRange)) for x in xRange]

def plotMissionTimes():
	pass
