
# python imports
import os
import time
import math
import subprocess
import numpy as np
import traceback
from pprint import pprint
from dataclasses import dataclass
from scipy.stats import norm
from scipy.spatial.distance import euclidean
from math import sqrt, prod
from multiprocessing import Manager, Pool, get_context
from contextlib import nullcontext

# project imports
from .NodeUtils import *
from .GlnsUtils import *
from .Planner import Planner
from .Constants import gtspInputFilename, gtspOutputFilename
from .PartitionSolver import ExtendedPartitionSolver
from .TspUtils import solveTspWithFixedStartEnd

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
    def __sub__(self, other): return Cost(*[x[0] - x[1] for x in zip(self.value, other.value)])
    def __mul__(self, constant): return Cost(*[x * constant for x in self.value])
    def __lt__(self, other): return self.value < other.value
    def __le__(self, other): return self.value <= other.value
    def __eq__(self, other): return self.value == other.value
    def __ne__(self, other): return self.value != other.value
    def __ge__(self, other): return self.value >= other.value
    def __gt__(self, other): return self.value > other.value

def safeExp(f):
	"""Returns a safe exponential that will not overflow / underflow"""
	try:
		return math.exp(f)
	except OverflowError:
		return float('inf') if f > 0 else 0.0

class OurPlanner(Planner):
    """Defines a planner class which implements our planning algorithm"""

    INF = 999999
    GTSP_SCALING = 100 # glns wants integers
    COST_DIM = 0 # dimension of cost

    def __init__(self, params):
        super().__init__(params)
        self.costMatrix = {} # str(agentType) : matrix[startIndex][endIndex]

    def standardizeSolution(self):
        result = {k:v for k,v in self.solution.items()}
        result['uav_points'] = [list(v) for v in result['uav_points']] # tuple -> list
        result['ugv_point_map'] = {k:[float(n) for n in v] for k,v in result["ugv_point_map"].items()} # yaml does not like np.array
        result['tour_costs'] = {k:[[float(c) for c in e.value] for e in v] for k,v in result['tour_costs'].items()} # Cost -> list
        result['tour_constraint_values'] = {k:[float(c) for c in l] for k,l in result['tour_constraint_values'].items()} # numpy scalar -> float
        return result

    def createCostMatrix(self, points, agentType = ''):
        """Fills a cost matrix for list of tuples"""
        if self.env == None:
            print('No planning environment, defaulting to distance for TSP')
            costMatrix = createDistanceMatrix(points)
        else:
            costMatrix = createFunctionMatrix(points,
                lambda point1, point2 : self.env.estimateMean(point1, point2, agentType)
            )

        self.costMatrix[agentType] = costMatrix

    def reorderCostMatrix(self, newOrder, agentType):
        """Reorders the cost matrix"""
        self.costMatrix[agentType] = createSubmatrix(self.costMatrix[agentType], newOrder)

    def baseCost(self, agentType = ''):
        """Sets a base cost for an agent"""
        raise NotImplementedError("OurPlanner subclass must implement baseCost(self, agentType))")

    def constructCost(self, p1, p2, agentType = ''):
        """Constructs a cost for an agent traveling from index p1 to index p2"""
        raise NotImplementedError("OurPlanner subclass must implement constructCost(self, p1, p2, agentType))")

    def evaluateConstraintBoolean(self, cost, agentType = ''):
        """Returns whether a cost is feasible"""
        raise NotImplementedError("OurPlanner subclass must implement evaluateConstraintBoolean(self, cost, agentType)")

    def evaluateConstraintsBoolean(self, costDict):
        """Returns whether all {agentType : agentCost, ...} constraints are satisfied"""
        return all(self.evaluateConstraintBoolean(agentCost, agentType) for agentType, agentCost in costDict.items())

    def evaluateConstraintFloat(self, cost, agentType = ''):
        """Returns a float to represent the constraint state for a given cost, s.t. worse is more negative"""
        raise NotImplementedError("OurPlanner subclass must implement evaluateConstraintFloat(self, cost, agentType)")

    def evaluateConstraintsFloat(self, costDict):
        """Returns a float to represent the combination of constraints for {agentType : agentCost, ...}"""
        raise NotImplementedError("OurPlanner subclass must implement evaluateConstraintsFloat(self, costDict")

    def solve(self, points, startPoint=None, endPoint=None):
        """Solves a uav/ugv path for a set of points"""

        if startPoint is None:
            startPoint = points[0]
        if endPoint is None:
            endPoint = points[0]

        try:
            totalStartTime = time.perf_counter()

            # Solve TSP
            tspStartTime = time.perf_counter()
            self.createCostMatrix(points, 'UAV')
            uavPoints, newOrdering = self.solveUavGlobalTsp(points, startPoint, endPoint)
            tspEndTime = time.perf_counter()

            # Calculate cost matrices for uavPoints order
            # TODO memoize these matrices
            costStartTime = time.perf_counter()
            self.reorderCostMatrix(newOrdering, 'UAV')
            self.createCostMatrix(uavPoints, 'UGV')
            costEndTime = time.perf_counter()

            # Break into UAV tours
            tourStartTime = time.perf_counter()
            tours = self.createUavTours(uavPoints)
            tourEndTime = time.perf_counter()

            # Solve for UGV tours (release and collect)
            # and optionally refine in stoch case
            ugvStartTime = time.perf_counter()
            ugvResults, tourCollectCosts = self.createUgvTours(tours, uavPoints, startPoint, endPoint)
            ugvEndTime = time.perf_counter()

            # Move collect and release points to start/end of UAV tours
            tourRefineStartTime = time.perf_counter()
            tours, tourCollectCosts = self.refineTours(tours, uavPoints, ugvResults, tourCollectCosts)
            tourRefineEndTime = time.perf_counter()

            totalEndTime = time.perf_counter()

            # Save predicted costs to output
            tourCosts = {
                k : [self.baseCost('UAV')]*len(tours)              # agentType : [tour1cost, ...]
                for k in next(iter( tourCollectCosts[0].values() )).keys()    # for each agent type
            }
            tourConstraintValues = {
                k : [0]*len(tours)         # agentType : [tour1value, ...]
                for k in tourCosts.keys()   # for each agent type
            }
            # TODO this should really be parameterized on release/collect, but the way i'm constructing tourCollectCosts for the stochastic case avoids that complication by fixing release to what was chosen
            for iTour in range(len(tours)):
                collectPoint = ugvResults['ugv_point_map'][ugvResults['ugv_path'][2 + 2*iTour]]
                closestUavPointIndex = findClosestPoint(uavPoints, (*collectPoint[:2], 0))
                for agentType, agentCost in tourCollectCosts[iTour][closestUavPointIndex].items():
                    tourCosts[agentType][iTour] = agentCost
                    tourConstraintValues[agentType][iTour] = self.evaluateConstraintFloat(agentCost, agentType)

            # Save runtimes
            tspTime = tspEndTime - tspStartTime
            costTime = costEndTime - costStartTime
            tourTime = tourEndTime - tourStartTime
            ugvTime = ugvEndTime - ugvStartTime # for stoch
            if 'gtsp_solver_time' in ugvResults and ugvResults['gtsp_solver_time'] is not None:
                ugvTime = ugvResults['gtsp_solver_time'] # for det
            refineTime = tourRefineEndTime - tourRefineStartTime
            totalTime = totalEndTime - totalStartTime # this will be the sum of the other times + julia startup time (in gtsp step)

            self.timeInfo = {
                "TSP_TIME": tspTime,
                "COST_TIME": costTime,
                "TOUR_TIME": tourTime,
                "UGV_TIME": ugvTime,
                "REFINE_TIME": refineTime,
                "TOTAL_TIME": totalTime
            }

            self.solution = ugvResults
            self.solution.pop('gtsp_solver_time', None)
            self.solution.update({
				'uav_points': uavPoints,
				'uav_tours': tours,
                'tour_costs': tourCosts,
                'tour_constraint_values': tourConstraintValues
				})

        except Exception:
            print("\nFailure during planning")
            print(traceback.format_exc())

    def solveUavGlobalTsp(self, uavPoints, startPoint, endPoint):
        """
        Solves a TSP across uavPoints in mean cost
        without regard to limits
        """

        # Find the closest points to START_POINT and END_POINT
        # TODO these should be cost matrix calls
        startIndex = findClosestPoint(uavPoints, (*startPoint, 0))
        endIndex = findClosestPoint(uavPoints, (*endPoint, 0))

        # Reorder so the closest points to start and end are at the beginning and end
        reorderedPointOrder = reorderList(list(range(len(uavPoints))), startIndex, endIndex)

        # Create the distance matrix for the reordered points
        costMatrix = createSubmatrix(self.costMatrix['UAV'], reorderedPointOrder)

        # Solve TSP
        localSearch = 'TSP_LOCAL_SEARCH' in self.params and self.params['TSP_LOCAL_SEARCH']
        newReorderedPointOrder = solveTspWithFixedStartEnd(
            costMatrix, 0, len(uavPoints) - 1, localSearch=localSearch
        )

        # Feed TSP solution through existing reordering
        newPointOrder = [reorderedPointOrder[i] for i in newReorderedPointOrder]
        newUavPoints = [uavPoints[i] for i in newPointOrder]

        return newUavPoints, newPointOrder

    def createUavTours(self, tspPlan):
        raise NotImplementedError("OurPlanner subclass must implement createUavTours(self, tspPlan)")

    def createUgvTours(self, tours, uavPoints, startPoint, endPoint):
        raise NotImplementedError("OurPlanner subclass must implement createUgvTours(self, tours, uavPoints, startPoint, endPoint)")

    def refineTours(self, tours, uavPoints, ugvResults, tourCollectCosts):
        """Refines UAV tours by moving release and collect points to start and end"""
        for iTour in range(len(tours)):
            # release
            releasePoint = ugvResults['ugv_point_map'][ugvResults['ugv_path'][1 + 2*iTour]]
            closestUavReleaseIndex = findClosestPoint(uavPoints, (*releasePoint[:2], 0) )
            tourReleaseIndex = tours[iTour].index(closestUavReleaseIndex)
            # collect
            collectPoint = ugvResults['ugv_point_map'][ugvResults['ugv_path'][2 + 2*iTour]]
            closestUavCollectIndex = findClosestPoint(uavPoints, (*collectPoint[:2], 0) )
            tourCollectIndex = tours[iTour].index(closestUavCollectIndex)
            # reorder
            tours[iTour] = reorderList(tours[iTour], tourReleaseIndex, tourCollectIndex)
        return tours, tourCollectCosts

class OurPlannerDeterministic(OurPlanner):
    """Only considers mean travel time"""

    def __init__(self, params):
        super().__init__(params)
        self.COST_DIM = 1

    def baseCost(self, agentType=''):
        """Construct a cost tuple representing 0"""
        if agentType == 'UAV':
            return Cost(self.params['TAKEOFF_LANDING_TIME'])
        elif agentType == 'UGV':
            return Cost(0)
        raise IndexError('agentType %s not recognized'%agentType)

    def constructCost(self, p1, p2, agentType=''):
        """Construct a cost tuple from p1 to p2"""
        return Cost(self.costMatrix[agentType][p1, p2])

    def evaluateConstraintBoolean(self, cost, agentType = ''):
        """Returns whether the tour constraint is satisfied"""
        if agentType == 'UAV':
            return 0 > self.params['UAV_DELTA_TIME'] - self.evaluateConstraintFloat(cost, agentType)
        elif agentType == 'UGV':
            return 0 > self.params['UGV_DELTA_TIME'] - self.evaluateConstraintFloat(cost, agentType)
        raise IndexError('agentType %s not recognized'%agentType)

    def evaluateConstraintFloat(self, cost, agentType = ''):
        """Returns (limit - mean) in time"""
        return self.params['UAV_BATTERY_TIME'] - cost.value[0]

    def evaluateConstraintsFloat(self, costDict):
        """Returns worst (limit - mean) in time for all agent types"""
        return min(self.evaluateConstraintsFloat(agentCost, agentType) for agentType, agentCost in costDict.items())

    def closeTour(self, currentTour, prevIndex, tourStartIndex, currentCost):
        """Find a valid collect point for closing a tour, while maximizing cost"""

        bestReturnCost = self.baseCost('UAV')
        bestUavCost = self.baseCost('UAV')
        bestIndex = None

        for candidateIndex in currentTour:
            returnCost = self.constructCost(prevIndex, candidateIndex, 'UAV')
            ugvCost = self.constructCost(tourStartIndex, candidateIndex, 'UGV')
            uavCost = currentCost + returnCost
            costDict = {'UAV' : uavCost, 'UGV' : ugvCost}

            if uavCost > bestUavCost:
                # constraints are UAV time and UGV time against battery
                if self.evaluateConstraintsBoolean(costDict):
                    bestUavCost = uavCost
                    bestReturnCost = returnCost
                    bestIndex = candidateIndex

        print(f"  --> Closing at best collect {bestIndex}, returnTime={bestReturnCost}")
        return bestReturnCost

    def createUavTours(self, tspPlan):
        """Finds feasible UAV tours within full TSP solution"""

        tours = []
        tourCosts = []
        currentTour = [0]
        currentCost = self.baseCost('UAV')
        tourStartIndex = 0
        prevIndex = 0

        print(f"\n=== START TOUR 0 ===")
        print(f"Start point index: 0, coords: {tspPlan[prevIndex]}")

        for currIndex in range(1, len(tspPlan)):
            travelCost = self.constructCost(currIndex, prevIndex, 'UAV')

            print(f"\n-- Considering point {currIndex} {tspPlan[currIndex]}")
            print(f" Current tour: {currentTour}")
            print(f" Travel time prev->curr: {travelCost}")
            print(f" CurrentTime before: {currentCost}")

            success = False
            for candidateIndex in currentTour + [currIndex]:
                returnCost = self.constructCost(currIndex, candidateIndex, 'UAV')
                ugvCost = self.constructCost(tourStartIndex, candidateIndex, 'UGV')
                uavCost = currentCost + travelCost + returnCost
                costDict = {'UAV' : uavCost, 'UGV' : ugvCost}

                print(f" Candidate collect {candidateIndex}: UAV arrival={uavCost}, UGV time={ugvCost}")

                if self.evaluateConstraintsBoolean(costDict):
                    success = True
                    print(f" Feasible collect point {candidateIndex}")
                    break

            if success:
                currentTour.append(currIndex)
                currentCost += travelCost
                prevIndex = currIndex
                print(f"  --> Accepted point {currIndex}, new currentTime={currentCost}")
            else:
                print(f" Point {currIndex} would break tour -> CLOSE current tour")

                # close tour
                bestReturnCost = self.closeTour(currentTour, prevIndex, tourStartIndex, currentCost)
                currentCost += bestReturnCost

                tours.append(currentTour)
                tourCosts.append(currentCost)

                print(f" Tour closed: {currentTour}, total time={currentCost}")
                print(f"=== START TOUR {len(tours)} ===")
                print(f" Start point index: {currIndex}, coords: {tspPlan[currIndex]}")

                currentTour = [currIndex]
                tourStartIndex = currIndex
                currentCost = self.baseCost('UAV')
                prevIndex = currIndex

        # Close final tour
        print(f"\n*** Closing final tour ***")
        bestReturnCost = self.closeTour(currentTour, prevIndex, tourStartIndex, currentCost)
        currentCost += bestReturnCost

        tours.append(currentTour)
        tourCosts.append(currentCost)
        print(f"Final tour closed: {currentTour}, total time={currentCost}")

        print("\nTours created CAHIT:")
        pprint(tours)
        print(f"Tour times CAHIT = {tourCosts}")
        return tours

    def createUgvTours(self, tours, uavPoints, startPoint, endPoint):
        """
        Chooses a UGV path by solving a GTSP
        where the release point of each tour is the first point
        and the collect point is any feasible point in that tour
        """
        # Compute collection costs
        tourCollectCosts = self.computeAllTourCosts(
            tours,
            uavPoints
        )

        # Collapse tour costs to worst mean cost per agent type
        minTourCollectCosts = [ {
            tourCollectPoint : min(agentCost for agentCost in tourCosts.values())
            for tourCollectPoint, tourCosts in tourCollectCosts[iTour].items()
        } for iTour in range(len(tourCollectCosts)) ]

        # Choose UGV tour
        gtspResult = self.solveGtspWithReleaseCollect(
            points=uavPoints,
            tours=tours,
            tourCollectCosts=minTourCollectCosts,
            startPoint=startPoint,
            endPoint=endPoint
        )

        missionTime = gtspResult["total_cost"] / self.GTSP_SCALING
        print("Total mission time (s):", missionTime)

        return gtspResult, tourCollectCosts

    def computeAllTourCosts(self, tours, points):
        """
        Computes UAV times for all tours and all possible collect points in each tour.
        Adds fixed takeoff and landing time to each.
        Checks feasibility against UAV battery and UGV travel limits.
        Returns one list:
            collectPoints: list of dicts mapping only feasible collectIdx to UAV time
        """

        points = np.array(points)

        tourCollectCosts = []
        for _, tour in enumerate(tours):
            thisTourCollectCosts = {}
            travelCost = sum( [self.constructCost(tour[k+1], tour[k], 'UAV') for k in range(len(tour) - 1)], Cost(*[0]*self.COST_DIM) )

            for collectIdx in tour:
                if collectIdx == tour[-1]:
                    totalCost = travelCost
                else:
                    extraCost = self.constructCost(collectIdx, tour[-1], 'UAV')
                    totalCost = travelCost + extraCost

                uavCost = totalCost + self.baseCost('UAV')
                ugvCost = self.constructCost(tour[0], collectIdx, 'UGV')
                costDict = {'UAV' : uavCost, 'UGV' : ugvCost}

                if not self.evaluateConstraintsBoolean(costDict):
                    continue

                thisTourCollectCosts[collectIdx] = costDict

            tourCollectCosts.append(thisTourCollectCosts)

        print("Collect points:")
        pprint(tourCollectCosts)
        return tourCollectCosts

    def buildtGTSPMatrix(self, mappingToRelease, mappingToCollect, points, tours, collectToTourCosts, dim, startPoint, endPoint):
        """
        Builds a distance matrix and cluster grouping list for UGV's GTSP solution
        """

        CHARGE_RATE = self.params["CHARGE_RATE"]

        matrix = np.ones((dim, dim)) * self.INF

        if self.env is None:
            startCost = euclidean(startPoint, points[tours[0][0]])
        else:
            startCost = self.env.estimateMean(startPoint, points[tours[0][0]], 'UGV')
        matrix[0, mappingToRelease[tours[0][0]]] = startCost
        matrix[mappingToRelease[tours[0][0]], 0] = startCost

        # set up dummy point that only attaches to start and end
        matrix[dim-1, 0] = 0
        matrix[0, dim-1] = 0
        matrix[dim-2, dim-1] = 0
        matrix[dim-1, dim-2] = 0

        clusters = [[0],[dim-1],[dim-2]]

        for tourIndex in range(0, len(tours)):
            tour = tours[tourIndex]
            collectCostsDict = collectToTourCosts[tourIndex]
            releaseIdx = tour[0]
            clusters.append([mappingToRelease[releaseIdx]])
            collectCluster = []
            for collectIdx, tourCost in collectCostsDict.items():
                collectGraphIdx = mappingToCollect[collectIdx]
                collectCluster.append(collectGraphIdx)
                matrix[collectGraphIdx, mappingToRelease[releaseIdx]] = tourCost.value[0]
                matrix[mappingToRelease[releaseIdx], collectGraphIdx] = tourCost.value[0]

                if tourIndex < len(tours) - 1:
                    nextReleaseIdx = tours[tourIndex + 1][0]
                    nextReleaseGraphIdx = mappingToRelease[nextReleaseIdx]
                    collectReleaseCost = max( self.constructCost(collectIdx, nextReleaseIdx, 'UGV').value[0], tourCost.value[0] * CHARGE_RATE )
                    matrix[collectGraphIdx, nextReleaseGraphIdx] = collectReleaseCost
                    matrix[nextReleaseGraphIdx, collectGraphIdx] = collectReleaseCost
            clusters.append(collectCluster)

        for collectIdx in collectToTourCosts[-1].keys():
            if self.env is None:
                endCost = euclidean(points[collectIdx], endPoint)
            else:
                endCost = self.env.estimateMean(points[collectIdx], endPoint, 'UGV')
            matrix[mappingToCollect[collectIdx], dim-2] = endCost
            matrix[dim-2, mappingToCollect[collectIdx]] = endCost

        matrix *= self.GTSP_SCALING
        matrix = matrix.astype(int)
        print(f"Clusters: {clusters}")
        return matrix, clusters

    def solveGtspWithReleaseCollect(self, points, tours, tourCollectCosts, startPoint, endPoint):
        """
        Solves the UGV's GTSP problem: chooses a release and collect point for each UAV tour in points
        """

        KSI = self.params["KSI"]
        runFolder = self.params["RUN_FOLDER"]

        points = np.array(points)[:, :2]
        graphIndex = 1
        mappingToRelease = {}
        mappingToCollect = {}
        mappingToPoints = {0: startPoint}
        # TODO: Use graphx
        for tour, collectCostsDict in zip(tours, tourCollectCosts):
            releaseIdx = tour[0]
            mappingToRelease[releaseIdx] = graphIndex
            mappingToPoints[graphIndex] = points[releaseIdx]
            graphIndex += 1
            for collectIdx in collectCostsDict.keys():
                mappingToCollect[collectIdx] = graphIndex
                mappingToPoints[graphIndex] = points[collectIdx]
                graphIndex += 1

        mappingToPoints[graphIndex] = endPoint
        mappingToPoints[graphIndex+1] = endPoint # Dummy Point - only connected to end point, to guarantee the tour ends here
        graphIndex += 2
        dim = graphIndex

        print(f"Mapping to release:")
        pprint(mappingToRelease)
        print(f"Mapping to collect:")
        pprint(mappingToCollect)

        distanceMatrix, clusters = self.buildtGTSPMatrix(mappingToRelease, mappingToCollect, points, tours, tourCollectCosts, dim, startPoint, endPoint)
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

        # remove dummy point (repeated end point)
        dummyIdx = path[-1]
        path = path[:-1]
        mappingToPoints.pop(dummyIdx, None)

        return {
            "ugv_path": path,
            "gtsp_solver_time": solverTime,
            "total_cost": totalCost,
            "ugv_point_map": mappingToPoints # TODO this is now a holdover and may be replacable with a more uav-like representation of points
        }

class OurPlannerStochastic(OurPlanner):
    """Considers mean and variance travel time, as calculated from an environmental model"""

    def __init__(self, params):
        super().__init__(params)
        self.costVarMatrix = {} # str(agentType) : matrix[startIndex][endIndex]
        self.COST_DIM = 2
        self.logSuccessLimit = math.log(1 - self.params['FAILURE_RISK'])

    def printResultsToYaml(self, maxDecimals=4):
        return super().printResultsToYaml(maxDecimals)

    def createCostMatrix(self, points, agentType=''):
        """Create mean AND variance matrices"""
        super().createCostMatrix(points, agentType)

        if self.env == None:
            print('No planning environment, defaulting to 0 for variance')
            numPoints = len(points)
            costVarMatrix = np.zeros((numPoints, numPoints))
        else:
            costVarMatrix = createFunctionMatrix(points,
                lambda point1, point2 : self.env.estimateVariance(point1, point2, agentType)
            )

        self.costVarMatrix[agentType] = costVarMatrix

    def reorderCostMatrix(self, newOrder, agentType):
        """Reorder mean AND variance matrices"""
        super().reorderCostMatrix(newOrder, agentType)
        self.costVarMatrix[agentType] = createSubmatrix(self.costVarMatrix[agentType], newOrder)

    def baseCost(self, agentType=''):
        """Construct a cost tuple representing 0"""
        if agentType == 'UAV':
            return Cost(self.params['TAKEOFF_LANDING_TIME'], 0) # TODO this should have variance; should add to plan_settings or just consult the environmental model
        elif agentType == 'UGV':
            return Cost(0, 0)
        raise IndexError('agentType %s not recognized'%agentType)

    def constructCost(self, p1 : int, p2 : int, agentType=''):
        """Construct a cost tuple from p1 to p2"""
        return Cost(self.costMatrix[agentType][p1, p2], self.costVarMatrix[agentType][p1, p2])

    def evaluateConstraintBoolean(self, cost, agentType = ''):
        """Returns whether the tour constraint is satisfied"""
        return self.evaluateConstraintFloat(cost, agentType) > self.logSuccessLimit

    def evaluateConstraintFloat(self, cost, agentType=''):
        """Returns the log chance of a (mean, var) time not exceeding the limit"""
        limit = self.params['UAV_BATTERY_TIME']
        mean = cost.value[0]
        var = cost.value[1]
        return norm.logsf( (-limit + mean) / sqrt(var) ) if var > 0 \
            else 0 if mean < limit \
            else -float('inf')

    def evaluateConstraintsFloat(self, costDict):
        """Returns risk of independent failures for all agentTypes"""
        return 1 - prod(1 - self.evaluateConstraintsFloat(agentCost, agentType) for agentType, agentCost in costDict.items())

    def logSuccessChanceUavUgv(self, uavCost, tourStartIndex, tourEndIndex):
        """
        Returns log chance of both uav and ugv cost not exceeding the limit
        by sweeping release and collect points
        Note that because this is written for use with the PartitionSolver, which has n+1 costs, tourStartIndex is inclusive and tourEndIndex is exclusive
        Note also that in case of symmetry e.g. Pr(success | r,c) = Pr(success | c,r) we choose c > r to minimize UGV transit time between tours
        """

        baseTour = np.arange(tourStartIndex, tourEndIndex)

        tourLength = tourEndIndex - tourStartIndex # length of tour = N
        rcIndices = np.arange(tourLength) # 0:N
        r, c = np.meshgrid(rcIndices, rcIndices) # 2d arrays of 0:N
        rcPairs = np.array([r.flatten(), c.flatten()]).T # [[r,c], ...] for each combination

        tours = np.apply_along_axis( # for each r,c pair
            lambda a :
            np.concatenate((
                reorderList(baseTour, a[0], a[1]), # reorder the tour
                np.zeros( ( int(a[0] != a[1]), ), dtype=np.int8 ) - 1 # padded with a -1 if r != c
            ), 0),
            1, rcPairs
        )

        logSuccessChances = np.apply_along_axis(
            lambda t:
            self.evaluateConstraintFloat( sum(
                [self.constructCost(t[i], t[i+1], 'UAV') for i in range(tours.shape[1] - 1) if t[i+1] >= 0],
                start = self.baseCost('UAV')
            ), 'UAV' ) + # UAV log success chance
            self.evaluateConstraintFloat(
                self.constructCost(t[0], t[-1] if t[-1] >= 0 else t[-2], 'UGV') +
                self.baseCost('UGV'),
                'UGV' ), # plus UGV log success chance
            1, tours # for each tour
        )

        bestOption = np.argmax(logSuccessChances) # find max index
        bestRelease, bestCollect = rcPairs[bestOption, :] # best release and collect
        bestLogChance = logSuccessChances[bestOption] # best log success chance

        # check for symmetry
        if bestRelease > bestCollect and logSuccessChances[bestRelease * tourLength + bestCollect] == bestLogChance:
            bestRelease, bestCollect = bestCollect, bestRelease

        # chance, (release global index, collect global index)
        return bestLogChance, (tourStartIndex + bestRelease, tourStartIndex + bestCollect)

    def createUavTours(self, tspPlan):
        """
        Break TSP solution into feasible uav tours
        and optionally refine by computing a TSP on each
        """
        # Construct tour cost per leg
        #   we only have to keep a running total of UAV because UGV just goes from release to collect
        tourCosts = [Cost(0,0)] + [ # this represents point 0 -> point 0
            self.constructCost(i, i+1, 'UAV') for i in range(len(tspPlan) - 1)]

        self.partitionSolver = ExtendedPartitionSolver(tourCosts, self.logSuccessChanceUavUgv, concave=False)
        parallelize = 'PARALLEL_DP' in self.params and self.params['PARALLEL_DP']
        if parallelize:
            print('Parallelizing DP solver')

        with Manager() if parallelize else nullcontext() as manager, get_context("spawn").Pool() if parallelize else nullcontext() as pool:
            for numSegments in range(1, len(tspPlan) + 1):
                print(f'Partitioning for {numSegments} tours')
                cuts, totalLogSuccess, segmentLogSuccesses = self.partitionSolver.solvePartitionProblem(numSegments, manager=manager, pool=pool)
                print(f'\tFound log(Pr(success)) = {totalLogSuccess:.2e} -> Pr(success) = {safeExp(totalLogSuccess):.4f}')
                if safeExp(totalLogSuccess) > 1 - self.params['FAILURE_RISK']:
                    print('\tAccepting')
                    break

        assert safeExp(totalLogSuccess) > 1 - self.params['FAILURE_RISK'], 'No feasible UAV tours found'

        # construct tours
        cuts = [0] + cuts + [len(tspPlan)]
        tours = [ list(range(cuts[i], cuts[i+1])) for i in range(len(cuts)-1) ]

        return tours

    def createUgvTours(self, tours, uavPoints, startPoint, endPoint):
        """
        Since we are solving for release and collect in createUavTours,
        we just need to pull the data out of the solver
        """
        ugvResults = {
            'ugv_path': list(range(2 + 2*len(tours))), # [ point indices ]
            'ugv_point_map': {}, # { index : [x,y], ... }
        }
        tourCollectCosts = [] # [ {collect point: {'agentType': collect cost, ...}, ...} for each tour ]

        # starting point, ending point, dummy point
        ugvResults['ugv_point_map'].update({
            0 : startPoint,
            1 + 2 * len(tours) : endPoint
        })

        # each tour
        for i in range(len(tours)):
            tour = tours[i]
            # Ask partitionSolver cache for release and collect
            startEndTuple = (tour[0], tour[-1] + 1) # inclusive, exclusive
            release, collect = self.partitionSolver.fData[startEndTuple]
            ugvResults['ugv_point_map'].update({
                2 * i + 1 : uavPoints[release][:2],
                2 * i + 2 : uavPoints[collect][:2]
            })
            reorderedTour = reorderList(tour, tour.index(release), tour.index(collect))
            tourCollectCosts.append({collect :
            {
                'UAV': sum([
                    self.constructCost(reorderedTour[i], reorderedTour[i+1], 'UAV') for i in range(len(reorderedTour)-1)
                    ], start = self.baseCost('UAV')),
                'UGV': self.constructCost(release, collect, 'UGV') + self.baseCost('UGV')
            }
            })

        print('Consulting partition solver for UGV tour:')
        for i in ugvResults['ugv_path']:
            print(f'\t{i} : {ugvResults["ugv_point_map"][i]}')

        return ugvResults, tourCollectCosts

    def refineTour(self, tour):
        """Refines a single tour with a TSP given a release and collect"""
        newTour = []
        if len(tour) > 3:
            thisTourCostMatrix = createSubmatrix(self.costMatrix['UAV'], tour)
            newTourIndices = solveTspWithFixedStartEnd(
                thisTourCostMatrix, 0, len(tour) - 1,
            )
            newTour = [tour[j] for j in newTourIndices]
        return newTour

    def refineTours(self, tours, uavPoints, ugvResults, tourCollectCosts):
        """Moves release and collect in tours and optionally solves a TSP on each"""
        # move release and collect to front and back
        tours, tourCollectCosts = super().refineTours(tours, uavPoints, ugvResults, tourCollectCosts)

        # then solve a TSP
        if 'REFINE_TOURS' in self.params and self.params['REFINE_TOURS']:
            print('Refining tours for time')

            # # evaluating TSPs in parallel seems to hang
            # with get_context("spawn").Pool(len(tours)) as tspPool:
            # tspPool = get_context("spawn").Pool(len(tours))
                # newTours = tspPool.map(self.refineTour, tours, chunksize=1)
            # tspPool.close()
            newTours = [self.refineTour(t) for t in tours]

            # test each for acceptance
            for i in range(len(tours)):
                if len(tours[i]) > 3:
                    newTour = newTours[i]
                    # calculate costs and probabilities
                    existingUavCost = tourCollectCosts[i][tours[i][-1]]['UAV']
                    existingLogSuccess = self.evaluateConstraintFloat(existingUavCost)
                    newUavCost = sum([
                        self.constructCost(newTour[j], newTour[j+1], 'UAV') for j in range(len(newTour)-1)
                    ], start = self.baseCost('UAV'))
                    newLogSuccess = self.evaluateConstraintFloat(newUavCost)
                    # check if UAV Pr(success) has not decreased and mean time has not increased
                    if newLogSuccess >= existingLogSuccess and newUavCost.value[0] < existingUavCost.value[0]:
                        print(f'Improved tour {i}')
                        tours[i] = list(newTour)
                        tourCollectCosts[i][tours[i][-1]]['UAV'] = newUavCost
                    else:
                        print(f'Did not improve tour {i}')
                    print(
                        f'\tCost : {existingUavCost} -> {newUavCost}\n' +
                        f'\tlog(Pr(success)) : {existingLogSuccess:.2e} -> {newLogSuccess:.2e}\n' +
                        f'\tPr(success) : {safeExp(existingLogSuccess):.4f} -> {safeExp(newLogSuccess):.4f}'
                    )
                else:
                    print(f'Skipping tour {i} with length {len(tours[i])}')

        return tours, tourCollectCosts
