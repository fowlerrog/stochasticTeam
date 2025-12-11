# python imports
import numpy as np
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

def solveTspWithFixedStartEnd(costMatrix : np.array, startIndex, endIndex, localSearch=False):
	"""Solves a TSP with a fixed start and end point"""

	# Create the routing index manager, setting start and end locations correctly
	manager = pywrapcp.RoutingIndexManager(
		costMatrix.shape[0],  # Number of locations
		1,  # Number of vehicles
		[startIndex],  # Start location index (closest to start)
		[endIndex]  # End location index (closest to end)
	)

	# Create the routing model
	routing = pywrapcp.RoutingModel(manager)

	# Define cost of each arc
	def costCallback(fromIndex, toIndex):
		fromNode = manager.IndexToNode(fromIndex)
		toNode = manager.IndexToNode(toIndex)
		return costMatrix[fromNode][toNode]

	transitCallbackIndex = routing.RegisterTransitCallback(costCallback)
	routing.SetArcCostEvaluatorOfAllVehicles(transitCallbackIndex)

	# Setting first solution heuristic
	searchParameters = pywrapcp.DefaultRoutingSearchParameters()
	searchParameters.first_solution_strategy = (
		routing_enums_pb2.FirstSolutionStrategy.CHRISTOFIDES)
		# routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION)

	# Set local search metaheuristic to GUIDED_LOCAL_SEARCH
	if localSearch:
		searchParameters.local_search_metaheuristic = (
			routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
		searchParameters.time_limit.seconds = 5

	# Solve the problem
	solution = routing.SolveWithParameters(searchParameters)

	tspAirPoints = []  # To store air points (excluding start and end)

	# Extracting solution
	if solution:
		index = routing.Start(0)
		while not routing.IsEnd(index):
			nodeIndex = manager.IndexToNode(index)
			tspAirPoints.append(nodeIndex)  # Adjusted for reordered points

			index = solution.Value(routing.NextVar(index))

		# Add the last point to the TSP tour and air points
		nodeIndex = manager.IndexToNode(index)
		tspAirPoints.append(nodeIndex)

	return tspAirPoints

	# TODO allow for asymmetric TSP (tree doubling)
