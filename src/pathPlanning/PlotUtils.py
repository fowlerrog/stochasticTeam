# python imports
import os
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
from math import log

# project imports
from .Constants import planPathResultsFilename
from .RunnerUtils import toDir, loadYamlContents, sigFigs

def plotPoints(points, filename=None, show=True, startPoint=None, endPoint=None, simple=False, pointColor='gray', startEndColor='blue', fig=None, ax=None):
	"""Plot points with no path"""
	xs = [p[0] for p in points]
	ys = [p[1] for p in points]

	if fig is None or ax is None:
		fig, ax = plt.subplots(figsize=(4, 4))
	ax.plot(xs, ys, 'o', markersize=8, color=pointColor)

	plt.axis('equal')
	if simple:
		ax.set_xticks([])
		ax.set_yticks([])
	else:
		plt.title('P_uav')
		plt.grid(True)

	if startPoint is not None:
		ax.plot(startPoint[0], startPoint[1], '^', color=startEndColor, markersize=16)
	if endPoint is not None:
		ax.plot(endPoint[0], endPoint[1], 'v', color=startEndColor, markersize=16)

	plt.tight_layout()
	if filename is not None:
		plt.savefig(filename)
	if show:
		plt.show()

	return fig, ax

def plotPath(points, filename=None, show=True, startPoint=None, endPoint=None, simple=False, pointColor='blue', startEndColor='blue', fig=None, ax=None):
	"""Plot points with path"""
	# Extract x,y (uses first two coordinates if points are 3D)
	xs = [p[0] for p in points]
	ys = [p[1] for p in points]

	# Plot the tour path with points
	if fig is None or ax is None:
		fig, ax = plt.subplots(figsize=(4, 4))
	ax.plot(xs, ys, color=pointColor, marker='o', markersize=8)
	for i in range(len(points) - 1):
		ax.annotate("", xy=(xs[i+1], ys[i+1]), xytext=(xs[i], ys[i]),
					arrowprops=dict(arrowstyle="->", color=pointColor, lw=2))

	# Mark start and end (first and last in the tour order)
	# ax.scatter(xs[0], ys[0], marker='s', s=100, label='Start (closest)')
	# ax.scatter(xs[-1], ys[-1], marker='X', s=120, label='End (closest)')

	ax.axis('equal')
	if simple:
		ax.set_xticks([])
		ax.set_yticks([])
	else:
		ax.set_title('TSP Tour with Fixed Start/End (reordered points)')
		ax.grid(True)

	if startPoint is not None:
		ax.plot(startPoint[0], startPoint[1], '^', color=startEndColor, markersize=16)
	if endPoint is not None:
		ax.plot(endPoint[0], endPoint[1], 'v', color=startEndColor, markersize=16)

	plt.tight_layout()
	if filename is not None:
		plt.savefig(filename)
	if show:
		plt.show()

	return fig, ax

def plotTours(uavTours, uavPoints, ugvPointMap, ugvPath, filename=None, show=True, fig=None, ax=None, simple=False, pointColor='blue', ugvColor='gray', startEndColor=None):
	"""Plot points, clustered into tours"""
	tourColorGroups = [0] * len(uavPoints)
	for cix, tour in enumerate(uavTours):
		for p in tour:
			tourColorGroups[p] = cix

	if simple:
		tourColors = lambda i: pointColor
	else:
		tourColors = plt.cm.get_cmap('tab20', len(uavTours))
	if fig is None or ax is None:
		fig, ax = plt.subplots(figsize=(4, 4))
	# print(f"Mapping to points: {mapping_to_points}")
	uavPoints = np.array(uavPoints)[:, :2]

	# plot colored circles for uav points
	for i, point in enumerate(uavPoints):
		x,y = point
		# print(x, y, tourColorGroups[i], tourColors(tourColorGroups[i]))
		color = tourColors(tourColorGroups[i])
		ax.plot(x, y, 'o', color=color, markersize=8)
		if not simple:
			ax.text(x, y, f"{i}a", fontsize=16, color=color, verticalalignment='top')

	# plot colored circles for ugv path (including start, end)
	i = 0
	while i < len(ugvPath):
		x,y = ugvPointMap[ugvPath[i]]
		shape = '^' if i == 0 else 'v' if i == len(ugvPath) - 1 else 'o'
		if simple:
			color = startEndColor if startEndColor is not None and (i == 0 or i == len(ugvPath) - 1) else pointColor
			markersize = 16 if i == 0 or i == len(ugvPath) - 1 else 0
			ax.plot(x, y, shape, color=color, markersize=markersize)
		else:
			color = 'g' if i == 0 else 'r' if i == len(ugvPath) - 1 else 'k'
			markersize = 16 if i == 0 or i == len(ugvPath) - 1 else 8
			ax.plot(x, y, shape, color=color, markersize=markersize)
			# label (with repeated points)
			labelNum = str(i)
			while i < len(ugvPath) - 1 and ugvPointMap[ugvPath[i]] == ugvPointMap[ugvPath[i+1]]:
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
						arrowprops=dict(arrowstyle="->", color=pointColor, lw=2))

	# plot ugv path
	for i in range(len(ugvPath)-1):
		x1, y1 = ugvPointMap[ugvPath[i]]
		x2, y2 = ugvPointMap[ugvPath[(i + 1) % len(ugvPath)]]
		if x1 == x2 and y1 == y2:
			continue
		# print(f"Drawing edge from {ugvPath[i]} to {ugvPath[(i + 1) % len(ugvPath)]}: ({x1}, {y1}) to ({x2}, {y2})")
		ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
					arrowprops=dict(arrowstyle="->", color=ugvColor, lw=2))

	ax.relim()
	ax.autoscale_view()
	ax.set_aspect('equal')
	if simple:
		ax.set_xticks([])
		ax.set_yticks([])
	else:
		ax.grid(True)
		ax.set_title("Full Mission Plan")

	plt.tight_layout()
	if filename is not None:
		plt.savefig(filename)
	if show:
		plt.show()

	return fig, ax

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

def plotPlanFromPlanResults(filePath, full=False, simple=False):
	"""Generates plots for a TSP path from a results folder"""
	absFolderPath = toDir(filePath)

	plan = loadYamlContents(filePath, planPathResultsFilename)
	if plan is None:
		return

	if full:
		plotPoints(plan['uav_points'],
			startPoint=plan['ugv_point_map'][plan['ugv_path'][0]],
			endPoint=plan['ugv_point_map'][plan['ugv_path'][-1]],
			show=False, simple=simple)

		TSPFigureName = os.path.join(absFolderPath, 'TSP_path.png')
		if 'uav_points' in plan:
			plotPath(plan['uav_points'], filename=TSPFigureName, show=False,
				startPoint=plan['ugv_point_map'][plan['ugv_path'][0]],
				endPoint=plan['ugv_point_map'][plan['ugv_path'][-1]],
				simple=simple)

	toursFigureName = os.path.join(absFolderPath, 'uav_tours.png')
	plotTours(plan['uav_tours'],
			    plan['uav_points'],
				plan["ugv_point_map"],
				plan["ugv_path"],
				toursFigureName,
				False,
				simple=simple)

	plt.show()

def plotMultiTeamPlanFromPlanResults(filePaths, full=False, simple=False):
	"""Generates plots for a TSP path with multiple teams, from multiple results folders"""

	# store figures and axes for reuse
	figs = [None] * 4
	axs = [None] * 4

	# define team colors
	numTeams = len(filePaths)
	colors = colorblindPalette()
	teamColors = [colors[i % len(colors)] for i in range(numTeams)]
	grayness = 0.3
	teamUgvColors = [(*(rgb * 0.5 * (1 - grayness) + 0.5 for rgb in c[:3]), c[3]) for c in teamColors] # muted

	for i in range(numTeams):
		# load each file
		filePath = filePaths[i]
		absFolderPath = toDir(filePath)
		plan = loadYamlContents(filePath, planPathResultsFilename)
		if plan is None:
			continue

		# plot points and TSP if desired
		if full:
			figs[0], axs[0] = plotPoints(plan['uav_points'],
				startPoint=plan['ugv_point_map'][plan['ugv_path'][0]],
				endPoint=plan['ugv_point_map'][plan['ugv_path'][-1]],
				show=False, simple=simple,
				fig=figs[0], ax=axs[0],
				pointColor='gray', startEndColor='k')

			figs[3], axs[3] = plotPoints(plan['uav_points'],
				startPoint=plan['ugv_point_map'][plan['ugv_path'][0]],
				endPoint=plan['ugv_point_map'][plan['ugv_path'][-1]],
				show=False, simple=simple,
				fig=figs[3], ax=axs[3],
				pointColor=teamColors[i], startEndColor='k')

			TSPFigureName = os.path.join(absFolderPath, 'multi_TSP_path.png') if i == len(filePaths) - 1 else None
			if 'uav_points' in plan:
				figs[1], axs[1] = plotPath(plan['uav_points'], filename=TSPFigureName, show=False,
					startPoint=plan['ugv_point_map'][plan['ugv_path'][0]],
					endPoint=plan['ugv_point_map'][plan['ugv_path'][-1]],
					simple=simple,
					fig=figs[1], ax=axs[1],
					pointColor=teamColors[i], startEndColor='k')

		toursFigureName = os.path.join(absFolderPath, 'multi_uav_tours.png') if i == len(filePaths) - 1 else None
		figs[2], axs[2] = plotTours(plan['uav_tours'],
					plan['uav_points'],
					plan["ugv_point_map"],
					plan["ugv_path"],
					toursFigureName,
					False,
					simple=simple,
					fig=figs[2], ax=axs[2],
					pointColor=teamColors[i], ugvColor=teamUgvColors[i], startEndColor='k')

	plt.show()

def colorblindPalette():
	colors = [
		'#1f77b4',  # blue
		'#ff7f0e',  # orange
		'#2ca02c',  # green
		'#d62728',  # red
		'#9467bd',  # purple
		'#8c564b',  # brown
		'#e377c2',  # pink
		'#7f7f7f',  # gray
		'#bcbd22',  # olive
		'#17becf'   # cyan
	]
	return [
		(*(int('0x' + c[2*rgb+1 : 2*rgb+3], 16) / 255.0 for rgb in range(3)), 1.0)
		for c in colors
	]

def plotSelfComparison(plannedValues, empiricalValues, labelString='', varName='', log=False, fig=None, ax=None):
	"""Plots planned values against empirical values"""
	# create figure if none given
	if fig is None or ax is None:
		fig, ax = plt.subplots()

	# get data limits excluding nans
	propLineMin = np.nanmax([np.nanmin(plannedValues), np.nanmin(empiricalValues)])
	propLineMax = np.nanmin([np.nanmax(plannedValues), np.nanmax(empiricalValues)])

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
			'min' : np.nanmin(thisY),
			'mean' : np.nanmean(thisY),
			'max' : np.nanmax(thisY),
			'std' : np.nanstd(thisY),
		}

	# print data
	print('Statistics:')
	for k in sorted(dataDict.keys()):
		v = dataDict[k]
		print(f"n={k}: min {v['min']:.2f} mean {v['mean']:.2f} max {v['max']:.2f} -> mu sigma {sigFigs(v['mean'],2)} +- {sigFigs(v['std'],2)}")

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
	xLimits = [np.nanmin(xSorted), np.nanmax(xSorted)]
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
	return [fOfXMin * f(x) / f(np.nanmin(xRange)) for x in xRange]
