
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
from RunnerUtils import writeJson

class OurPlanner(Planner):
    """Defines a planner class which implements our planning algorithm"""

    INF = 999999
    GTSP_SCALING = 100 # glns wants integers

    def solve(self, points):
        """Solves a path for a set of points"""

        try:
            # Solve TSP
            tsp_start_time = time.perf_counter()
            uav_points = self.solve_tsp_with_fixed_start_end(points)
            tsp_end_time = time.perf_counter()

            # Break into UAV cycles
            cycles = self.create_cycles_CAHIT(uav_points)

            # Solve UGV GTSP
            collect_to_cycle_times = self.compute_all_cycle_times(
                cycles,
                uav_points
            )
            result = self.solve_gtsp_with_release_collect(
                points=uav_points,
                cycles=cycles,
                collect_to_cycle_times=collect_to_cycle_times,
            )

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

    def print_results_to_json(self):
        """Prints solution results to a json file"""
        # construct save dict
        result = self.solution
        result['ugv_mapping_to_points'] = {k:list(v) for k,v in result["ugv_mapping_to_points"].items()} # json does not like np.array
        absSavePath = os.path.join(self.params["RUN_FOLDER"], self.params["SAVE_PATH_FOLDER"], planPathResultsFilename)
        writeJson(result, absSavePath)

    def solve_tsp_with_fixed_start_end(self, points):
        """Solves a TSP with a fixed start and end point"""
        start_point = self.params["START_POINT"]
        end_point = self.params["END_POINT"]

        # Step 1: Find the closest points to START_POINT and END_POINT
        start_index = find_closest_point(points, start_point)
        end_index = find_closest_point(points, end_point)

        # Step 2: Reorder the points with the closest points to start and end at the beginning and end
        reordered_points = reorder_list(points, start_index, end_index)

        # Step 3: Create the distance matrix for the reordered points
        distance_matrix = create_distance_matrix(reordered_points)

        print("\nDISTANCE MATRIX:")
        print(distance_matrix)
        # exit()
        # Step 4: Create the routing index manager, setting start and end locations correctly
        manager = pywrapcp.RoutingIndexManager(
            len(distance_matrix),  # Number of locations
            1,  # Number of vehicles
            [0],  # Start location index (closest to start)
            [len(distance_matrix) - 1]  # End location index (closest to end)
        )

        # Step 5: Create the routing model
        routing = pywrapcp.RoutingModel(manager)

        # Step 6: Define cost of each arc
        def distance_callback(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return distance_matrix[from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Step 7: Setting first solution heuristic
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.CHRISTOFIDES)
            # routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION)

        # COMMENT IF ONLY WANT TO USE THE FIRST SOL
        # Set local search metaheuristic to GUIDED_LOCAL_SEARCH
        # search_parameters.local_search_metaheuristic = (
        #     routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
        # search_parameters.time_limit.seconds = 5

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

    def close_cycle(self, current_cycle, tsp_tour, prev_point, cycle_start_point, current_time):
        """Find best collect point for closing a cycle"""
        UAV_SPEED = self.params["UAV_SPEED"]
        UGV_SPEED = self.params["UGV_SPEED"]
        UAV_BATTERY_TIME = self.params["UAV_BATTERY_TIME"]

        best_return_time = float('inf')
        best_uav_time = -float('inf')
        best_j = None

        for j in current_cycle:
            candidate = tsp_tour[j]
            return_time = euclidean(prev_point, candidate) / UAV_SPEED
            ugv_time = euclidean(cycle_start_point, candidate) / UGV_SPEED
            uav_time = current_time + return_time

            # new logic for ugv only checks battery
            if uav_time <= UAV_BATTERY_TIME and ugv_time <= UAV_BATTERY_TIME:
                if uav_time > best_uav_time:
                    best_uav_time = uav_time
                    best_return_time = return_time
                    best_j = j

        print(f"  --> Closing at best collect {best_j}, return_time={best_return_time:.2f}")
        return best_return_time

    def create_cycles_CAHIT(self, tsp_tour):
        TAKEOFF_LANDING_TIME = self.params["TAKEOFF_LANDING_TIME"]
        UAV_SPEED = self.params["UAV_SPEED"]
        UGV_SPEED = self.params["UGV_SPEED"]
        UAV_BATTERY_TIME = self.params["UAV_BATTERY_TIME"]

        cycles = []
        cycle_times = []
        current_cycle = [0]
        current_time = TAKEOFF_LANDING_TIME
        cycle_start_point = tsp_tour[0]
        prev_point = tsp_tour[0]

        print(f"\n=== START CYCLE 0 ===")
        print(f"Start point index: 0, coords: {prev_point}")

        for i in range(1, len(tsp_tour)):
            curr_point = tsp_tour[i]
            travel_time = euclidean(curr_point, prev_point) / UAV_SPEED

            print(f"\n-- Considering point {i} {curr_point}")
            print(f" Current cycle: {current_cycle}")
            print(f" Travel time prev->curr: {travel_time:.2f}")
            print(f" Current_time before: {current_time:.2f}")

            success = False
            for j in current_cycle + [i]:
                candidate = tsp_tour[j]
                return_time = euclidean(curr_point, candidate) / UAV_SPEED
                ugv_time = euclidean(cycle_start_point, candidate) / UGV_SPEED
                uav_time = current_time + travel_time + return_time

                print(f" Candidate collect {j}: UAV arrival={uav_time:.2f}, UGV time={ugv_time:.2f}")

                if uav_time <= UAV_BATTERY_TIME and ugv_time <= UAV_BATTERY_TIME:
                    success = True
                    print(f" Feasible collect point {j}")
                    break

            if success:
                current_cycle.append(i)
                current_time += travel_time
                prev_point = curr_point
                print(f"  --> Accepted point {i}, new current_time={current_time:.2f}")
            else:
                print(f" Point {i} would break cycle -> CLOSE current cycle")

                # close cycle
                best_return_time = self.close_cycle(current_cycle, tsp_tour, prev_point, cycle_start_point, current_time)
                current_time += best_return_time
                cycles.append(current_cycle)
                cycle_times.append(current_time)

                print(f" Cycle closed: {current_cycle}, total time={current_time:.2f}")
                print(f"=== START CYCLE {len(cycles)} ===")
                print(f" Start point index: {i}, coords: {curr_point}")

                current_cycle = [i]
                cycle_start_point = tsp_tour[i]
                current_time = TAKEOFF_LANDING_TIME
                prev_point = curr_point

        # Close final cycle
        print(f"\n*** Closing final cycle ***")
        best_return_time = self.close_cycle(current_cycle, tsp_tour, prev_point, cycle_start_point, current_time)

        current_time += best_return_time
        cycles.append(current_cycle)
        cycle_times.append(current_time)
        print(f"Final cycle closed: {current_cycle}, total time={current_time:.2f}")

        print("\nCycles created CAHIT:")
        pprint(cycles)
        print(f"Cycle times CAHIT = {cycle_times}")
        return cycles

    def compute_all_cycle_times(self, cycles, points):
        """
        Computes UAV times for all cycles and all possible collect points in each cycle.
        Adds fixed takeoff and landing time to each.
        Checks feasibility against UAV battery and UGV travel limits.
        Returns one list:
            collect_points: list of dicts mapping only feasible collect_idx to UAV time
        """
        UAV_SPEED = self.params["UAV_SPEED"]
        UGV_SPEED = self.params["UGV_SPEED"]
        TAKEOFF_LANDING_TIME = self.params["TAKEOFF_LANDING_TIME"]
        UAV_BATTERY_TIME = self.params["UAV_BATTERY_TIME"]

        points = np.array(points)
    
        collect_points = []
        for _, cycle in enumerate(cycles):
            collect_to_cycle_time = {}
            #TODO first and last should be different
            travel_dist = sum(np.linalg.norm(points[cycle[k+1]] - points[cycle[k]]) for k in range(len(cycle) - 1))

            for collect_idx in cycle:
                collect_point = points[collect_idx]
                if collect_idx == cycle[-1]:
                    total_dist = travel_dist
                else:
                    last_point = points[cycle[-1]]
                    #TODO project the below collect
                    extra_dist = np.linalg.norm(collect_point - last_point)
                    total_dist = travel_dist + extra_dist

                time_sec = total_dist / UAV_SPEED + TAKEOFF_LANDING_TIME
                if time_sec > UAV_BATTERY_TIME:
                    continue

                ugv_release = points[cycle[0]]
                ugv_collect = points[collect_idx]
                #TODO below should be projected
                ugv_dist = np.linalg.norm(ugv_collect - ugv_release)
                ugv_time = ugv_dist / UGV_SPEED

                if ugv_time > UAV_BATTERY_TIME:
                    continue

                collect_to_cycle_time[collect_idx] = time_sec

                #TODO
                #total_dist = total_dist - return_dist + last_uav_travel_actual_dist


            collect_points.append(collect_to_cycle_time)

        print("Collect points:")
        pprint(collect_points)

        return collect_points

    def buildt_GTSP_matrix(self, mapping_to_release, mapping_to_collect, points, cycles, collect_to_cycle_times, dim):
        """
        Builds a distance matrix and cluster grouping list for UGV's GTSP solution
        """
        start_point = self.params["START_POINT"]
        end_point = self.params["END_POINT"]

        UGV_SPEED = self.params["UGV_SPEED"]
        CHARGE_RATE = self.params["CHARGE_RATE"]

        matrix = np.ones((dim, dim)) * self.INF

        first_release_point = points[cycles[0][0]]

        matrix [0,mapping_to_release[cycles[0][0]]] = euclidean(first_release_point, start_point) / UGV_SPEED
        matrix [mapping_to_release[cycles[0][0]], 0] = euclidean(first_release_point, start_point) / UGV_SPEED

        matrix[dim-1, 0] = 0
        matrix[0, dim-1] = 0
        matrix[dim-2, dim-1] = 0
        matrix[dim-1, dim-2] = 0

        clusters = [[0],[dim-1],[dim-2]]

        for cycle_index in range(0, len(cycles)):
            cycle = cycles[cycle_index]
            collect_times_dict = collect_to_cycle_times[cycle_index]
            release_idx = cycle[0]
            clusters.append([mapping_to_release[release_idx]])
            collect_cluster = []
            for collect_idx, cycle_time in collect_times_dict.items():
                collect_graph_idx = mapping_to_collect[collect_idx]
                collect_point = points[collect_idx]
                collect_cluster.append(collect_graph_idx)
                matrix[collect_graph_idx, mapping_to_release[release_idx]] = cycle_time
                matrix[mapping_to_release[release_idx], collect_graph_idx] = cycle_time

                if cycle_index < len(cycles) - 1:
                    next_release_idx = cycles[cycle_index + 1][0]
                    next_release_graph_idx = mapping_to_release[next_release_idx]
                    next_release_point = points[next_release_idx]
                    matrix[collect_graph_idx, next_release_graph_idx] = max(euclidean(collect_point, next_release_point) / UGV_SPEED, cycle_time * CHARGE_RATE)
                    matrix[next_release_graph_idx, collect_graph_idx] = max(euclidean(collect_point, next_release_point) / UGV_SPEED, cycle_time * CHARGE_RATE)
            clusters.append(collect_cluster)

        for collect_idx in collect_to_cycle_times[-1].keys():
            matrix[mapping_to_collect[collect_idx], dim-2] = euclidean(points[collect_idx], end_point) / UGV_SPEED
            matrix[dim-2, mapping_to_collect[collect_idx]] = euclidean(points[collect_idx], end_point) / UGV_SPEED

        matrix *= self.GTSP_SCALING
        matrix = matrix.astype(int)
        print(f"Clusters: {clusters}")
        return matrix, clusters

    def solve_gtsp_with_release_collect(self, points, cycles, collect_to_cycle_times):
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
        for cycle, collect_times_dict in zip(cycles, collect_to_cycle_times):
            release_idx = cycle[0]
            mapping_to_release[release_idx] = graph_index
            mapping_to_points[graph_index] = points[release_idx]
            graph_index += 1
            for collect_idx in collect_times_dict.keys():
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

        distance_matrix, clusters = self.buildt_GTSP_matrix(mapping_to_release, mapping_to_collect, points, cycles, collect_to_cycle_times, dim)
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
            "clusters": clusters,
            "ugv_mapping_to_points": mapping_to_points
        }
