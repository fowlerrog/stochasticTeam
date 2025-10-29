
# python imports
import os
import time
import subprocess
import numpy as np
import traceback
from pprint import pprint
from scipy.spatial.distance import euclidean
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

# project imports
from NodeUtils import *
from GtspUtils import *
from Planner import Planner
from Constants import gtspInputFilename, gtspOutputFilename, planPathResultsFilename
from RunnerUtils import writeYaml

class OurPlanner(Planner):
    """Defines a planner class which implements our planning algorithm"""

    INF = 999999
    GTSP_SCALING = 100 # glns wants integers

    def __init__(self, params):
        super().__init__(params)
        self.cost_matrix = {} # str(agentType) : matrix[startIndex][endIndex]

    def solve(self, points):
        """Solves a uav/ugv path for a set of points"""

        try:
            # Solve TSP
            tsp_start_time = time.perf_counter()
            uav_points = self.solve_tsp_with_fixed_start_end(points)
            tsp_end_time = time.perf_counter()

            # Calculate cost matrices for uav_points order
            self.create_cost_matrix(uav_points, 'UAV')
            self.create_cost_matrix(uav_points, 'UGV')

            # Break into UAV cycles
            cycles = self.create_cycles_CAHIT(uav_points)

            # Solve UGV GTSP
            collect_to_cycle_times = self.compute_all_cycle_costs(
                cycles,
                uav_points
            )
            result = self.solve_gtsp_with_release_collect(
                points=uav_points,
                cycles=cycles,
                collect_to_cycle_costs=collect_to_cycle_times,
            )

            # Add collect points to UAV cycles
            for iCycle in range(len(cycles)):
                collect_point = result['ugv_mapping_to_points'][result['ugv_path'][2 + 2*iCycle]]
                collect_point_projection = (*collect_point[:2], self.params['FIXED_Z'])
                closest_uav_point_index = find_closest_point(uav_points, collect_point_projection)
                if cycles[iCycle][-1] != closest_uav_point_index:
                    cycles[iCycle].append(closest_uav_point_index)

            # Save runtimes
            mission_time = result["total_cost"] / self.GTSP_SCALING
            print("Total mission time (s):", mission_time)

            tsp_time = tsp_end_time - tsp_start_time
            gtsp_time = result["gtsp_solver_time"] or -1  # fallback in case it's None
            cycle_tsp_total_time = -1  # or fill if you compute it elsewhere

            self.time_info = {
                "MISSION_TIME": mission_time,
                "TSP_TIME": tsp_time,
                "GTSP_TIME": gtsp_time,
                "CYCLE_TSP_TOTAL_TIME": cycle_tsp_total_time
            }

            self.solution = result
            self.solution.update({
				'uav_points': uav_points,
				'uav_cycles': cycles
				})

        except Exception:
            print("\nFailure during planning")
            print(traceback.format_exc())

    def print_results_to_yaml(self):
        """Prints solution results to a yaml file"""
        # construct save dict
        result = self.solution
        result['uav_points'] = [list(v) for v in result['uav_points']] # tuple -> list
        result['ugv_mapping_to_points'] = {k:[float(n) for n in v] for k,v in result["ugv_mapping_to_points"].items()} # yaml does not like np.array
        absSavePath = os.path.join(self.params["RUN_FOLDER"], self.params["SAVE_PATH_FOLDER"], planPathResultsFilename)
        writeYaml(result, absSavePath)

    def solve_tsp_with_fixed_start_end(self, points):
        """Solves a TSP with a fixed start and end point"""
        start_point = self.params["START_POINT"]
        end_point = self.params["END_POINT"]

        # Step 1: Find the closest points to START_POINT and END_POINT
        start_index = find_closest_point(points, (*start_point, 0))
        end_index = find_closest_point(points, (*end_point, 0))

        # Step 2: Reorder the points with the closest points to start and end at the beginning and end
        reordered_points = reorder_list(points, start_index, end_index)

        # Step 3: Create the distance matrix for the reordered points
        cost_matrix = self.create_cost_matrix(reordered_points, 'UAV')

        # print("\nCOST MATRIX:")
        # print(cost_matrix)

        # Step 4: Create the routing index manager, setting start and end locations correctly
        manager = pywrapcp.RoutingIndexManager(
            len(cost_matrix),  # Number of locations
            1,  # Number of vehicles
            [0],  # Start location index (closest to start)
            [len(cost_matrix) - 1]  # End location index (closest to end)
        )

        # Step 5: Create the routing model
        routing = pywrapcp.RoutingModel(manager)

        # Step 6: Define cost of each arc
        def cost_callback(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return cost_matrix[from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(cost_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Step 7: Setting first solution heuristic
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.CHRISTOFIDES)
            # routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION)

        # Set local search metaheuristic to GUIDED_LOCAL_SEARCH
        if 'TSP_LOCAL_SEARCH' in self.params and self.params['TSP_LOCAL_SEARCH']:
            search_parameters.local_search_metaheuristic = (
                routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
            search_parameters.time_limit.seconds = 5

        # Step 8: Solve the problem
        solution = routing.SolveWithParameters(search_parameters)

        tsp_tour = []
        tsp_air_points = []  # To store air points (excluding start and end)

        # Extracting solution
        if solution:
            index = routing.Start(0)
            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)
                tsp_tour.append(node_index)
                tsp_air_points.append(reordered_points[node_index])  # Adjusted for reordered points

                index = solution.Value(routing.NextVar(index))

            # Add the last point to the TSP tour and air points
            node_index = manager.IndexToNode(index)
            tsp_tour.append(node_index)
            tsp_air_points.append(reordered_points[node_index])

        return tsp_air_points

    def create_cost_matrix(self, points, agentType = ''):
        """Fills a cost matrix for list of tuples"""
        if self.env == None:
            print('No planning environment, defaulting to distance for TSP')
            cost_matrix = create_distance_matrix(points)
        else:
            num_points = len(points)
            cost_matrix = zeros((num_points, num_points))

            for i in range(num_points):
                for j in range(num_points):
                    cost_matrix[i][j] = self.env.estimateMean(points[i], points[j], agentType)

        self.cost_matrix[agentType] = cost_matrix
        return cost_matrix

    def close_cycle(self, current_cycle, prev_index, cycle_start_index, current_cost):
        """Find a valid collect point for closing a cycle"""
        UAV_BATTERY_TIME = self.params["UAV_BATTERY_TIME"]

        best_return_cost = float('inf')
        best_uav_cost = -float('inf')
        best_index = None

        for candidate_index in current_cycle:
            return_cost = self.cost_matrix['UAV'][prev_index][candidate_index]
            ugv_cost = self.cost_matrix['UGV'][cycle_start_index][candidate_index]
            uav_cost = current_cost + return_cost

            if uav_cost > best_uav_cost:
                # constraints are UAV time and UGV time against battery
                if uav_cost <= UAV_BATTERY_TIME and ugv_cost <= UAV_BATTERY_TIME:
                    best_uav_cost = uav_cost
                    best_return_cost = return_cost
                    best_index = candidate_index

        print(f"  --> Closing at best collect {best_index}, return_time={best_return_cost:.2f}")
        return best_return_cost

    def create_cycles_CAHIT(self, tsp_tour):
        TAKEOFF_LANDING_TIME = self.params["TAKEOFF_LANDING_TIME"]
        UAV_BATTERY_TIME = self.params["UAV_BATTERY_TIME"]

        cycles = []
        cycle_costs = []
        current_cycle = [0]
        current_cost = TAKEOFF_LANDING_TIME
        cycle_start_index = 0
        prev_index = 0

        print(f"\n=== START CYCLE 0 ===")
        print(f"Start point index: 0, coords: {tsp_tour[prev_index]}")

        for curr_index in range(1, len(tsp_tour)):
            travel_cost = self.cost_matrix['UAV'][curr_index][prev_index]

            print(f"\n-- Considering point {curr_index} {tsp_tour[curr_index]}")
            print(f" Current cycle: {current_cycle}")
            print(f" Travel time prev->curr: {travel_cost:.2f}")
            print(f" Current_time before: {current_cost:.2f}")

            success = False
            for candidate_index in current_cycle + [curr_index]:
                return_cost = self.cost_matrix['UAV'][curr_index][candidate_index]
                ugv_cost = self.cost_matrix['UGV'][cycle_start_index][candidate_index]
                uav_cost = current_cost + travel_cost + return_cost

                print(f" Candidate collect {candidate_index}: UAV arrival={uav_cost:.2f}, UGV time={ugv_cost:.2f}")

                if uav_cost <= UAV_BATTERY_TIME and ugv_cost <= UAV_BATTERY_TIME:
                    success = True
                    print(f" Feasible collect point {candidate_index}")
                    break

            if success:
                current_cycle.append(curr_index)
                current_cost += travel_cost
                prev_index = curr_index
                print(f"  --> Accepted point {curr_index}, new current_time={current_cost:.2f}")
            else:
                print(f" Point {curr_index} would break cycle -> CLOSE current cycle")

                # close cycle
                best_return_cost = self.close_cycle(current_cycle, prev_index, cycle_start_index, current_cost)
                current_cost += best_return_cost

                cycles.append(current_cycle)
                cycle_costs.append(current_cost)

                print(f" Cycle closed: {current_cycle}, total time={current_cost:.2f}")
                print(f"=== START CYCLE {len(cycles)} ===")
                print(f" Start point index: {curr_index}, coords: {tsp_tour[curr_index]}")

                current_cycle = [curr_index]
                cycle_start_index = curr_index
                current_cost = TAKEOFF_LANDING_TIME
                prev_index = curr_index

        # Close final cycle
        print(f"\n*** Closing final cycle ***")
        best_return_cost = self.close_cycle(current_cycle, prev_index, cycle_start_index, current_cost)
        current_cost += best_return_cost

        cycles.append(current_cycle)
        cycle_costs.append(current_cost)
        print(f"Final cycle closed: {current_cycle}, total time={current_cost:.2f}")

        print("\nCycles created CAHIT:")
        pprint(cycles)
        print(f"Cycle times CAHIT = {cycle_costs}")
        return cycles

    def compute_all_cycle_costs(self, cycles, points):
        """
        Computes UAV times for all cycles and all possible collect points in each cycle.
        Adds fixed takeoff and landing time to each.
        Checks feasibility against UAV battery and UGV travel limits.
        Returns one list:
            collect_points: list of dicts mapping only feasible collect_idx to UAV time
        """
        TAKEOFF_LANDING_TIME = self.params["TAKEOFF_LANDING_TIME"]
        UAV_BATTERY_TIME = self.params["UAV_BATTERY_TIME"]

        points = np.array(points)
    
        collect_points = []
        for _, cycle in enumerate(cycles):
            collect_to_cycle_cost = {}
            #TODO first and last should be different
            travel_cost = sum( self.cost_matrix['UAV'][cycle[k+1]][cycle[k]] for k in range(len(cycle) - 1) )

            for collect_idx in cycle:
                if collect_idx == cycle[-1]:
                    total_cost = travel_cost
                else:
                    #TODO project the below collect
                    extra_cost = self.cost_matrix['UAV'][collect_idx][cycle[-1]]
                    total_cost = travel_cost + extra_cost

                uav_cost = total_cost + TAKEOFF_LANDING_TIME
                if uav_cost > UAV_BATTERY_TIME:
                    continue

                #TODO below should be projected
                ugv_dist = self.cost_matrix['UGV'][cycle[0]][collect_idx]
                ugv_cost = ugv_dist

                if ugv_cost > UAV_BATTERY_TIME:
                    continue

                collect_to_cycle_cost[collect_idx] = uav_cost

                #TODO
                #total_dist = total_dist - return_dist + last_uav_travel_actual_dist

            collect_points.append(collect_to_cycle_cost)

        print("Collect points:")
        pprint(collect_points)

        return collect_points

    def buildt_GTSP_matrix(self, mapping_to_release, mapping_to_collect, points, cycles, collect_to_cycle_costs, dim):
        """
        Builds a distance matrix and cluster grouping list for UGV's GTSP solution
        """
        start_point = self.params["START_POINT"]
        end_point = self.params["END_POINT"]

        CHARGE_RATE = self.params["CHARGE_RATE"]

        matrix = np.ones((dim, dim)) * self.INF

        startCost = self.env.estimateMean(start_point, points[cycles[0][0]], 'UGV')
        matrix[0, mapping_to_release[cycles[0][0]]] = startCost
        matrix[mapping_to_release[cycles[0][0]], 0] = startCost

        matrix[dim-1, 0] = 0
        matrix[0, dim-1] = 0
        matrix[dim-2, dim-1] = 0
        matrix[dim-1, dim-2] = 0

        clusters = [[0],[dim-1],[dim-2]]

        for cycle_index in range(0, len(cycles)):
            cycle = cycles[cycle_index]
            collect_costs_dict = collect_to_cycle_costs[cycle_index]
            release_idx = cycle[0]
            clusters.append([mapping_to_release[release_idx]])
            collect_cluster = []
            for collect_idx, cycle_cost in collect_costs_dict.items():
                collect_graph_idx = mapping_to_collect[collect_idx]
                collect_cluster.append(collect_graph_idx)
                matrix[collect_graph_idx, mapping_to_release[release_idx]] = cycle_cost
                matrix[mapping_to_release[release_idx], collect_graph_idx] = cycle_cost

                if cycle_index < len(cycles) - 1:
                    next_release_idx = cycles[cycle_index + 1][0]
                    next_release_graph_idx = mapping_to_release[next_release_idx]
                    matrix[collect_graph_idx, next_release_graph_idx] = max( self.cost_matrix['UGV'][collect_idx][next_release_idx], cycle_cost * CHARGE_RATE )
                    matrix[next_release_graph_idx, collect_graph_idx] = max( self.cost_matrix['UGV'][collect_idx][next_release_idx], cycle_cost * CHARGE_RATE )
            clusters.append(collect_cluster)

        for collect_idx in collect_to_cycle_costs[-1].keys():
            endCost = self.env.estimateMean(points[collect_idx], end_point, 'UGV')
            matrix[mapping_to_collect[collect_idx], dim-2] = endCost
            matrix[dim-2, mapping_to_collect[collect_idx]] = endCost

        matrix *= self.GTSP_SCALING
        matrix = matrix.astype(int)
        print(f"Clusters: {clusters}")
        return matrix, clusters

    def solve_gtsp_with_release_collect(self, points, cycles, collect_to_cycle_costs):
        """
        Solves the UGV's GTSP problem: chooses a release and collect point for each UAV cycle in points
        """

        start_point = self.params["START_POINT"]
        end_point = self.params["END_POINT"]

        KSI = self.params["KSI"]
        runFolder = self.params["RUN_FOLDER"]

        points = np.array(points)[:, :2]
        graph_index = 1
        mapping_to_release = {}
        mapping_to_collect = {}
        mapping_to_points = {0: start_point}
        # TODO: Use graphx
        for cycle, collect_costs_dict in zip(cycles, collect_to_cycle_costs):
            release_idx = cycle[0]
            mapping_to_release[release_idx] = graph_index
            mapping_to_points[graph_index] = points[release_idx]
            graph_index += 1
            for collect_idx in collect_costs_dict.keys():
                mapping_to_collect[collect_idx] = graph_index
                mapping_to_points[graph_index] = points[collect_idx]
                graph_index += 1

        mapping_to_points[graph_index] = end_point
        mapping_to_points[graph_index+1] = self.params["DUMMY_POINT"]  # Dummy point
        graph_index += 2
        dim = graph_index

        print(f"Mapping to release:")
        pprint(mapping_to_release)
        print(f"Mapping to collect:")
        pprint(mapping_to_collect)

        distance_matrix, clusters = self.buildt_GTSP_matrix(mapping_to_release, mapping_to_collect, points, cycles, collect_to_cycle_costs, dim)
        gtsp_input_path = os.path.join(runFolder, gtspInputFilename)
        gtsp_output_path = os.path.join(runFolder, gtspOutputFilename)
        write_gtsp_file(dim, clusters, distance_matrix, gtsp_input_path)

        max_run_time = KSI * len(points) ** 3
        subprocess.run([
            "julia", "-e",
            f'using GLNS; solver("{escape_path(gtsp_input_path)}", output="{escape_path(gtsp_output_path)}", max_time={max_run_time}); solver("{escape_path(gtsp_input_path)}", output="{escape_path(gtsp_output_path)}", max_time={max_run_time})'
        ])

        tour, solver_time, total_cost = read_glns_output(gtsp_output_path)
        print(f"Tour: {tour}")

        path = tour_to_path(tour, start_idx=0, dummy_idx=dim-1)
        print(f"Path: {path}")

        return {
            "ugv_path": path,
            "gtsp_solver_time": solver_time,
            "total_cost": total_cost,
            "ugv_mapping_to_points": mapping_to_points
        }
