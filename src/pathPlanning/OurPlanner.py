
# python imports
import os
import time
import subprocess
import numpy as np
import traceback
from pprint import pprint
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from dataclasses import dataclass
from scipy.stats import norm
from math import sqrt

# project imports
from .NodeUtils import *
from .GtspUtils import *
from .Planner import Planner
from .Constants import gtspInputFilename, gtspOutputFilename, planPathResultsFilename
from .RunnerUtils import writeYaml

@dataclass
class Cost:
    """Class for a cost with an arbitrary number of values, each of which sum independently"""
    value: list

    def __init__(self, *value):
        """Constructor for C(a,b,c,d,...)"""
        self.value = [*value]

    def __str__(self):
        """To string method"""
        return 'Cost' + str(self.value)

    """Operators:"""
    def __add__(self, other): return Cost(*[sum(x) for x in zip(self.value, other.value)])
    def __mul__(self, constant): return Cost(*[x * constant for x in self.value])
    def __lt__(self, other): return self.value < other.value
    def __le__(self, other): return self.value <= other.value
    def __eq__(self, other): return self.value == other.value
    def __ne__(self, other): return self.value != other.value
    def __ge__(self, other): return self.value >= other.value
    def __gt__(self, other): return self.value > other.value

class OurPlanner(Planner):
    """Defines a planner class which implements our planning algorithm"""

    INF = 999999
    GTSP_SCALING = 100 # glns wants integers
    COST_DIM = 0 # dimension of cost

    def __init__(self, params):
        super().__init__(params)
        self.costMatrix = {} # str(agentType) : matrix[startIndex][endIndex]

    def printResultsToYaml(self):
        """Prints solution results to a yaml file"""
        # construct save dict
        result = self.solution
        result['uav_points'] = [list(v) for v in result['uav_points']] # tuple -> list
        result['ugv_mapping_to_points'] = {k:[float(n) for n in v] for k,v in result["ugv_mapping_to_points"].items()} # yaml does not like np.array
        absSavePath = os.path.join(self.params["RUN_FOLDER"], self.params["SAVE_PATH_FOLDER"], planPathResultsFilename)
        writeYaml(result, absSavePath, maxDecimals=2)

    def createCostMatrix(self, points, agentType = ''):
        """Fills a cost matrix for list of tuples"""
        if self.env == None:
            print('No planning environment, defaulting to distance for TSP')
            costMatrix = createDistanceMatrix(points)
        else:
            numPoints = len(points)
            costMatrix = zeros((numPoints, numPoints))

            for i in range(numPoints):
                for j in range(numPoints):
                    costMatrix[i][j] = self.env.estimateMean(points[i], points[j], agentType)

        self.costMatrix[agentType] = costMatrix

    def baseCost(self, agentType = ''):
        """Sets a base cost for an agent"""
        raise NotImplementedError("OurPlanner subclass must implement baseCost(self, agentType))")

    def constructCost(self, p1, p2, agentType = ''):
        """Constructs a cost for an agent traveling from index p1 to index p2"""
        raise NotImplementedError("OurPlanner subclass must implement constructCost(self, p1, p2, agentType))")

    def evaluateConstraint(self, cost, agentType = ''):
        """Returns whether a cost is feasible"""
        raise NotImplementedError("OurPlanner subclass must implement evaluateConstraint(self, cost, agentType)")

    def solve(self, points):
        """Solves a uav/ugv path for a set of points"""

        try:
            # Solve TSP
            tspStartTime = time.perf_counter()
            uavPoints = self.solveTspWithFixedStartEnd(points)
            tspEndTime = time.perf_counter()

            # Calculate cost matrices for uavPoints order
            self.createCostMatrix(uavPoints, 'UAV')
            self.createCostMatrix(uavPoints, 'UGV')

            # Break into UAV cycles
            cycles = self.createCyclesCAHIT(uavPoints)

            # Solve UGV GTSP
            collectToCycleTimes = self.computeAllCycleCosts(
                cycles,
                uavPoints
            )
            result = self.solveGtspWithReleaseCollect(
                points=uavPoints,
                cycles=cycles,
                collectToCycleCosts=collectToCycleTimes,
            )

            # Add collect points to UAV cycles
            for iCycle in range(len(cycles)):
                collectPoint = result['ugv_mapping_to_points'][result['ugv_path'][2 + 2*iCycle]]
                collectPointProjection = (*collectPoint[:2], self.params['FIXED_Z'])
                closestUavPointIndex = findClosestPoint(uavPoints, collectPointProjection)
                if cycles[iCycle][-1] != closestUavPointIndex:
                    cycles[iCycle].append(closestUavPointIndex)

            # Save runtimes
            missionTime = result["total_cost"] / self.GTSP_SCALING
            print("Total mission time (s):", missionTime)

            tspTime = tspEndTime - tspStartTime
            gtspTime = result["gtsp_solver_time"] or -1  # fallback in case it's None
            cycleTspTotalTime = -1  # or fill if you compute it elsewhere

            self.timeInfo = {
                "MISSION_TIME": missionTime,
                "TSP_TIME": tspTime,
                "GTSP_TIME": gtspTime,
                "CYCLE_TSP_TOTAL_TIME": cycleTspTotalTime
            }

            self.solution = result
            self.solution.update({
				'uav_points': uavPoints,
				'uav_cycles': cycles
				})

        except Exception:
            print("\nFailure during planning")
            print(traceback.format_exc())

    def solveTspWithFixedStartEnd(self, points):
        """Solves a TSP with a fixed start and end point"""
        startPoint = self.params["START_POINT"]
        endPoint = self.params["END_POINT"]

        # Step 1: Find the closest points to START_POINT and END_POINT
        startIndex = findClosestPoint(points, (*startPoint, 0))
        endIndex = findClosestPoint(points, (*endPoint, 0))

        # Step 2: Reorder the points with the closest points to start and end at the beginning and end
        reorderedPoints = reorderList(points, startIndex, endIndex)

        # Step 3: Create the distance matrix for the reordered points
        self.createCostMatrix(reorderedPoints, 'UAV')
        costMatrix = self.costMatrix['UAV']

        # print("\nCOST MATRIX:")
        # print(costMatrix)

        # Step 4: Create the routing index manager, setting start and end locations correctly
        manager = pywrapcp.RoutingIndexManager(
            len(costMatrix),  # Number of locations
            1,  # Number of vehicles
            [0],  # Start location index (closest to start)
            [len(costMatrix) - 1]  # End location index (closest to end)
        )

        # Step 5: Create the routing model
        routing = pywrapcp.RoutingModel(manager)

        # Step 6: Define cost of each arc
        def costCallback(fromIndex, toIndex):
            fromNode = manager.IndexToNode(fromIndex)
            toNode = manager.IndexToNode(toIndex)
            return costMatrix[fromNode][toNode]

        transitCallbackIndex = routing.RegisterTransitCallback(costCallback)
        routing.SetArcCostEvaluatorOfAllVehicles(transitCallbackIndex)

        # Step 7: Setting first solution heuristic
        searchParameters = pywrapcp.DefaultRoutingSearchParameters()
        searchParameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.CHRISTOFIDES)
            # routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION)

        # Set local search metaheuristic to GUIDED_LOCAL_SEARCH
        if 'TSP_LOCAL_SEARCH' in self.params and self.params['TSP_LOCAL_SEARCH']:
            searchParameters.local_search_metaheuristic = (
                routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
            searchParameters.time_limit.seconds = 5

        # Step 8: Solve the problem
        solution = routing.SolveWithParameters(searchParameters)

        tspTour = []
        tspAirPoints = []  # To store air points (excluding start and end)

        # Extracting solution
        if solution:
            index = routing.Start(0)
            while not routing.IsEnd(index):
                nodeIndex = manager.IndexToNode(index)
                tspTour.append(nodeIndex)
                tspAirPoints.append(reorderedPoints[nodeIndex])  # Adjusted for reordered points

                index = solution.Value(routing.NextVar(index))

            # Add the last point to the TSP tour and air points
            nodeIndex = manager.IndexToNode(index)
            tspTour.append(nodeIndex)
            tspAirPoints.append(reorderedPoints[nodeIndex])

        return tspAirPoints

    def closeCycle(self, currentCycle, prevIndex, cycleStartIndex, currentCost):
        """Find a valid collect point for closing a cycle, while maximizing cost"""

        bestReturnCost = self.baseCost('UAV')
        bestUavCost = self.baseCost('UAV')
        bestIndex = None

        for candidateIndex in currentCycle:
            returnCost = self.constructCost(prevIndex, candidateIndex, 'UAV')
            ugvCost = self.constructCost(cycleStartIndex, candidateIndex, 'UGV')
            uavCost = currentCost + returnCost

            if uavCost > bestUavCost:
                # constraints are UAV time and UGV time against battery
                if self.evaluateConstraint(uavCost, 'UAV') and self.evaluateConstraint(ugvCost, 'UGV'):
                    bestUavCost = uavCost
                    bestReturnCost = returnCost
                    bestIndex = candidateIndex

        print(f"  --> Closing at best collect {bestIndex}, returnTime={bestReturnCost}")
        return bestReturnCost

    def createCyclesCAHIT(self, tspTour):
        """Finds feasible UAV cycles within full TSP solution"""

        cycles = []
        cycleCosts = []
        currentCycle = [0]
        currentCost = self.baseCost('UAV')
        cycleStartIndex = 0
        prevIndex = 0

        print(f"\n=== START CYCLE 0 ===")
        print(f"Start point index: 0, coords: {tspTour[prevIndex]}")

        for currIndex in range(1, len(tspTour)):
            travelCost = self.constructCost(currIndex, prevIndex, 'UAV')

            print(f"\n-- Considering point {currIndex} {tspTour[currIndex]}")
            print(f" Current cycle: {currentCycle}")
            print(f" Travel time prev->curr: {travelCost}")
            print(f" CurrentTime before: {currentCost}")

            success = False
            for candidateIndex in currentCycle + [currIndex]:
                returnCost = self.constructCost(currIndex, candidateIndex, 'UAV')
                ugvCost = self.constructCost(cycleStartIndex, candidateIndex, 'UGV')
                uavCost = currentCost + travelCost + returnCost

                print(f" Candidate collect {candidateIndex}: UAV arrival={uavCost}, UGV time={ugvCost}")

                if self.evaluateConstraint(uavCost, 'UAV') and self.evaluateConstraint(ugvCost, 'UGV'):
                    success = True
                    print(f" Feasible collect point {candidateIndex}")
                    break

            if success:
                currentCycle.append(currIndex)
                currentCost += travelCost
                prevIndex = currIndex
                print(f"  --> Accepted point {currIndex}, new currentTime={currentCost}")
            else:
                print(f" Point {currIndex} would break cycle -> CLOSE current cycle")

                # close cycle
                bestReturnCost = self.closeCycle(currentCycle, prevIndex, cycleStartIndex, currentCost)
                currentCost += bestReturnCost

                cycles.append(currentCycle)
                cycleCosts.append(currentCost)

                print(f" Cycle closed: {currentCycle}, total time={currentCost}")
                print(f"=== START CYCLE {len(cycles)} ===")
                print(f" Start point index: {currIndex}, coords: {tspTour[currIndex]}")

                currentCycle = [currIndex]
                cycleStartIndex = currIndex
                currentCost = self.baseCost('UAV')
                prevIndex = currIndex

        # Close final cycle
        print(f"\n*** Closing final cycle ***")
        bestReturnCost = self.closeCycle(currentCycle, prevIndex, cycleStartIndex, currentCost)
        currentCost += bestReturnCost

        cycles.append(currentCycle)
        cycleCosts.append(currentCost)
        print(f"Final cycle closed: {currentCycle}, total time={currentCost}")

        print("\nCycles created CAHIT:")
        pprint(cycles)
        print(f"Cycle times CAHIT = {cycleCosts}")
        return cycles

    def computeAllCycleCosts(self, cycles, points):
        """
        Computes UAV times for all cycles and all possible collect points in each cycle.
        Adds fixed takeoff and landing time to each.
        Checks feasibility against UAV battery and UGV travel limits.
        Returns one list:
            collectPoints: list of dicts mapping only feasible collectIdx to UAV time
        """

        points = np.array(points)

        collectPoints = []
        for _, cycle in enumerate(cycles):
            collectToCycleCost = {}
            #TODO first and last should be different
            travelCost = sum( [self.constructCost(cycle[k+1], cycle[k], 'UAV') for k in range(len(cycle) - 1)], Cost(*[0]*self.COST_DIM) )

            for collectIdx in cycle:
                if collectIdx == cycle[-1]:
                    totalCost = travelCost
                else:
                    #TODO project the below collect
                    extraCost = self.constructCost(collectIdx, cycle[-1], 'UAV')
                    totalCost = travelCost + extraCost

                uavCost = totalCost + self.baseCost('UAV')
                if not self.evaluateConstraint(uavCost, 'UAV'):
                    continue

                #TODO below should be projected
                ugvCost = self.constructCost(cycle[0], collectIdx, 'UGV')

                if not self.evaluateConstraint(ugvCost, 'UGV'):
                    continue

                collectToCycleCost[collectIdx] = uavCost

                #TODO
                #totalDist = totalDist - returnDist + lastUavTravelActualDist

            collectPoints.append(collectToCycleCost)

        print("Collect points:")
        pprint(collectPoints)

        return collectPoints

    def buildtGTSPMatrix(self, mappingToRelease, mappingToCollect, points, cycles, collectToCycleCosts, dim):
        """
        Builds a distance matrix and cluster grouping list for UGV's GTSP solution
        """
        startPoint = self.params["START_POINT"]
        endPoint = self.params["END_POINT"]

        CHARGE_RATE = self.params["CHARGE_RATE"]

        matrix = np.ones((dim, dim)) * self.INF

        startCost = self.env.estimateMean(startPoint, points[cycles[0][0]], 'UGV')
        matrix[0, mappingToRelease[cycles[0][0]]] = startCost
        matrix[mappingToRelease[cycles[0][0]], 0] = startCost

        matrix[dim-1, 0] = 0
        matrix[0, dim-1] = 0
        matrix[dim-2, dim-1] = 0
        matrix[dim-1, dim-2] = 0

        clusters = [[0],[dim-1],[dim-2]]

        for cycleIndex in range(0, len(cycles)):
            cycle = cycles[cycleIndex]
            collectCostsDict = collectToCycleCosts[cycleIndex]
            releaseIdx = cycle[0]
            clusters.append([mappingToRelease[releaseIdx]])
            collectCluster = []
            for collectIdx, cycleCost in collectCostsDict.items():
                collectGraphIdx = mappingToCollect[collectIdx]
                collectCluster.append(collectGraphIdx)
                matrix[collectGraphIdx, mappingToRelease[releaseIdx]] = cycleCost.value[0]
                matrix[mappingToRelease[releaseIdx], collectGraphIdx] = cycleCost.value[0]

                if cycleIndex < len(cycles) - 1:
                    nextReleaseIdx = cycles[cycleIndex + 1][0]
                    nextReleaseGraphIdx = mappingToRelease[nextReleaseIdx]
                    collectReleaseCost = max( self.constructCost(collectIdx, nextReleaseIdx, 'UGV').value[0], cycleCost.value[0] * CHARGE_RATE )
                    matrix[collectGraphIdx, nextReleaseGraphIdx] = collectReleaseCost
                    matrix[nextReleaseGraphIdx, collectGraphIdx] = collectReleaseCost
            clusters.append(collectCluster)

        for collectIdx in collectToCycleCosts[-1].keys():
            endCost = self.env.estimateMean(points[collectIdx], endPoint, 'UGV')
            matrix[mappingToCollect[collectIdx], dim-2] = endCost
            matrix[dim-2, mappingToCollect[collectIdx]] = endCost

        matrix *= self.GTSP_SCALING
        matrix = matrix.astype(int)
        print(f"Clusters: {clusters}")
        return matrix, clusters

    def solveGtspWithReleaseCollect(self, points, cycles, collectToCycleCosts):
        """
        Solves the UGV's GTSP problem: chooses a release and collect point for each UAV cycle in points
        """

        startPoint = self.params["START_POINT"]
        endPoint = self.params["END_POINT"]

        KSI = self.params["KSI"]
        runFolder = self.params["RUN_FOLDER"]

        points = np.array(points)[:, :2]
        graphIndex = 1
        mappingToRelease = {}
        mappingToCollect = {}
        mappingToPoints = {0: startPoint}
        # TODO: Use graphx
        for cycle, collectCostsDict in zip(cycles, collectToCycleCosts):
            releaseIdx = cycle[0]
            mappingToRelease[releaseIdx] = graphIndex
            mappingToPoints[graphIndex] = points[releaseIdx]
            graphIndex += 1
            for collectIdx in collectCostsDict.keys():
                mappingToCollect[collectIdx] = graphIndex
                mappingToPoints[graphIndex] = points[collectIdx]
                graphIndex += 1

        mappingToPoints[graphIndex] = endPoint
        mappingToPoints[graphIndex+1] = self.params["DUMMY_POINT"]  # Dummy point
        graphIndex += 2
        dim = graphIndex

        print(f"Mapping to release:")
        pprint(mappingToRelease)
        print(f"Mapping to collect:")
        pprint(mappingToCollect)

        distanceMatrix, clusters = self.buildtGTSPMatrix(mappingToRelease, mappingToCollect, points, cycles, collectToCycleCosts, dim)
        gtspInputPath = os.path.join(runFolder, gtspInputFilename)
        gtspOutputPath = os.path.join(runFolder, gtspOutputFilename)
        writeGtspFile(dim, clusters, distanceMatrix, gtspInputPath)

        maxRunTime = KSI * len(points) ** 3
        subprocess.run([
            "julia", "-e",
            f'using GLNS; solver("{escapePath(gtspInputPath)}", output="{escapePath(gtspOutputPath)}", max_time={maxRunTime}); solver("{escapePath(gtspInputPath)}", output="{escapePath(gtspOutputPath)}", max_time={maxRunTime})'
        ])

        tour, solverTime, totalCost = readGlnsOutput(gtspOutputPath)
        print(f"Tour: {tour}")

        path = tourToPath(tour, startIdx=0, dummyIdx=dim-1)
        print(f"Path: {path}")

        return {
            "ugv_path": path,
            "gtsp_solver_time": solverTime,
            "total_cost": totalCost,
            "ugv_mapping_to_points": mappingToPoints
        }

class OurPlannerDeterministic(OurPlanner):
    """Only considers mean travel time"""

    def __init__(self, params):
        super().__init__(params)
        self.COST_DIM = 1

    def baseCost(self, agentType=''):
        if agentType == 'UAV':
            return Cost(self.params['TAKEOFF_LANDING_TIME'])
        elif agentType == 'UGV':
            return Cost(0)
        raise IndexError('agentType %s not recognized'%agentType)

    def constructCost(self, p1, p2, agentType=''):
        return Cost(self.costMatrix[agentType][p1][p2])

    def evaluateConstraint(self, cost, agentType = ''):
        if agentType == 'UAV':
            return cost.value[0] < self.params['UAV_BATTERY_TIME'] - self.params['UAV_DELTA_TIME']
        elif agentType == 'UGV':
            return cost.value[0] < self.params['UAV_BATTERY_TIME'] - self.params['UGV_DELTA_TIME']
        raise IndexError('agentType %s not recognized'%agentType)

class OurPlannerStochastic(OurPlanner):
    """Considers mean and variance travel time, as calculated from an environmental model"""

    def __init__(self, params):
        super().__init__(params)
        self.costVarMatrix = {} # str(agentType) : matrix[startIndex][endIndex]
        self.COST_DIM = 2

    def createCostMatrix(self, points, agentType=''):
        """Create mean AND variance matrices"""
        super().createCostMatrix(points, agentType)

        numPoints = len(points)
        costVarMatrix = zeros((numPoints, numPoints))
        if self.env == None:
            print('No planning environment, defaulting to 0 for variance')
        else:
            for i in range(numPoints):
                for j in range(numPoints):
                    costVarMatrix[i][j] = self.env.estimateVariance(points[i], points[j], agentType)

        self.costVarMatrix[agentType] = costVarMatrix

    def constructCost(self, p1, p2, agentType=''):
        return Cost(self.costMatrix[agentType][p1][p2], self.costVarMatrix[agentType][p1][p2])

    def baseCost(self, agentType=''):
        if agentType == 'UAV':
            return Cost(self.params['TAKEOFF_LANDING_TIME'], 0)
        elif agentType == 'UGV':
            return Cost(0, 0)
        raise IndexError('agentType %s not recognized'%agentType)

    def evaluateConstraint(self, cost, agentType = ''):
        limit = self.params['UAV_BATTERY_TIME']
        risk = self.params['FAILURE_RISK']
        mean = cost.value[0]
        var = cost.value[1]

        return norm.cdf( (-limit + mean) / sqrt(var) ) < risk if var > 0 else mean < limit
