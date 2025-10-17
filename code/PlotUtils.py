import matplotlib.pyplot as plt
import numpy as np

def plot_points(points, filename=None, show=True):
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

def plot_path(points, filename=None, show=True):
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

def plot_clusters(cycles, clusters, points, mapping_to_points, path, filename=None, show=True):
	"""Plot points, clustered into cycles"""
	clusters_for_plot = []
	for cix, cycle in enumerate(cycles):
		clusters_for_plot.extend([cix]*len(cycle))

	cluster_colors = plt.cm.get_cmap('tab20', len(clusters))
	fig, ax = plt.subplots(figsize=(8, 6))
	# print(f"Mapping to points: {mapping_to_points}")
	points = np.array(points)[:, :2]
	for i, point in enumerate(points):
		x,y = point
		# print(x,y)
		color = cluster_colors(clusters_for_plot[i])
		ax.plot(x, y, 'o', color=color, markersize=16)
		ax.text(x + 15, y + 15, f"{i}", fontsize=16, color=color)

	for i in range(len(points)):
		x1, y1 = points[i]
		x2, y2 = points[(i + 1) % len(points)]
		ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
					arrowprops=dict(arrowstyle="-", color='black', lw=2))
	for i in range(len(path)):
		x1, y1 = mapping_to_points[path[i]]
		x2, y2 = mapping_to_points[path[(i + 1) % len(path)]]
		if x1 == x2 and y1 == y2:
			continue
		print(f"Drawing edge from {path[i]} to {path[(i + 1) % len(path)]}: ({x1}, {y1}) to ({x2}, {y2})")
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

def plot_original_cycles(original_points, original_cycles, start_point=None, end_point=None, filename=None, show=True):
	colors = plt.cm.get_cmap('tab20', len(original_cycles))
	fig, ax = plt.subplots(figsize=(8, 6))
	for i, cycle in enumerate(original_cycles):
		color = colors(i)
		for pt_idx in cycle:
			x, y, _ = original_points[pt_idx]
			ax.plot(x, y, 'o', color=color)
			ax.text(x + 10, y + 10, f"{pt_idx}", fontsize=8, color=color)
		cycle_coords = [original_points[pt_idx][:2] for pt_idx in cycle]
		cycle_coords.append(cycle_coords[0])  # close the cycle
		for j in range(len(cycle_coords) - 1):
			x1, y1 = cycle_coords[j]
			x2, y2 = cycle_coords[j + 1]
			ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
						arrowprops=dict(arrowstyle="->", color=color, lw=1.5))
	if start_point:
		ax.plot(start_point[0], start_point[1], 's', color='black', markersize=8, label='Start')
		ax.text(start_point[0] + 10, start_point[1] + 10, 'Start', fontsize=9, color='black')
	if end_point:
		ax.plot(end_point[0], end_point[1], 'D', color='black', markersize=8, label='End')
		ax.text(end_point[0] + 10, end_point[1] + 10, 'End', fontsize=9, color='black')

	release_points = [original_points[cycle[0]] for cycle in original_cycles if len(cycle) > 0]
	for i in range(len(release_points) - 1):
		x1, y1 = release_points[i][:2]
		x2, y2 = release_points[i + 1][:2]
		ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
					arrowprops=dict(arrowstyle="->", color='gray', lw=2, ls='--'))

	ax.set_title("Original UAV Cycles + UGV Release Path")
	ax.set_aspect('equal')
	ax.grid(True)

	if filename is not None:
		plt.savefig(filename)
	if show:
		plt.show()

# TODO - add main?