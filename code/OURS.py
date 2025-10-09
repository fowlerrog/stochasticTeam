import os
import time
import random
import subprocess
import numpy as np
from pprint import pprint

#import matplotlib
#matplotlib.use('TkAgg')

from matplotlib import cm
import matplotlib.pyplot as plt
from scipy.spatial.distance import euclidean
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
 
INF = 999999
UAV_SPEED = 10.0
UGV_SPEED = 2.5
TAKEOFF_LANDING_TIME = 100.0
UAV_BATTERY_TIME = 600.0
START_POINT = (0, 0)
END_POINT = (4000, 4000)
DUMMY_POINT = (4000, 0)
SPACE_SIZE = 4000
CHARGE_RATE = 1
KSI = 1e5 # GLNS PARAMETER
SEEDS = [77]
# SEEDS = [77, 42, 12, 777, 888, 17, 21, 314, 2000, 93, 
        #  2496, 1377, 454, 935, 142, 1775, 2128, 2801, 
        #  2903, 2803, 214, 2360, 1114, 668, 1630]
# SEEDS = [77, 42, 12, 666, 777, 888, 17, 21, 314, 2000, 93]
# NS = [25, 50, 75, 100]
NS = [25]
UGV_SPEEDS = [2.5]
SPACE_SIZE = 4000
FIXED_Z = 500

def escape_path(p): return p.replace("\\", "\\\\")

def read_glns_output(path):
    with open(path, "r") as f:
        for line in f:
            if "Tour" in line and "[" in line:
                tour = eval(line.split(":")[1].strip())
                tour = [int(x)-1 for x in tour]
            if "Solver Time" in line:
                solver_time = float(line.split(":")[1].strip().split()[0])
            if "Tour Cost" in line:
                total_cost = float(line.split(":")[1].strip())
    return tour, solver_time, total_cost
 
def write_gtsp_file(index, clusters, distance_matrix, gtsp_input_path):
    lines = [
        "NAME : GLNS_RELEASE_COLLECT",
        "TYPE : GTSP",
        f"DIMENSION : {index}",
        f"GTSP_SETS : {len(clusters)}",
        "EDGE_WEIGHT_TYPE : EXPLICIT",
        "EDGE_WEIGHT_FORMAT : FULL_MATRIX",
        "EDGE_WEIGHT_SECTION"
    ]
    for row in distance_matrix:
        lines.append(" ".join(map(str, row)))
    lines.append("GTSP_SET_SECTION")
    for cid, cluster in enumerate(clusters, start=1):
        lines.append(f"{cid} " + " ".join(str(i + 1) for i in cluster) + " -1")
    lines.append("EOF")
    with open(gtsp_input_path, "w") as f:
        f.write("\n".join(lines))
 
def tour_to_path(tour, start_idx, dummy_idx):
    start_pos = tour.index(start_idx)
    dummy_pos = tour.index(dummy_idx)

    if (start_pos + 1) % len(tour) == dummy_pos:
        path = tour[start_pos+1:] + tour[:start_pos+1]
        path = path[::-1]
    elif (start_pos - 1) % len(tour) == dummy_pos:
        path = tour[start_pos:] + tour[:start_pos]
    else:
        raise ValueError("start_idx and dummy_idx are not adjacent in the tour")

    # path = [x for x in path if x != dummy_idx]

    return path

def buildt_GTSP_matrix(mapping_to_release, mapping_to_collect, points, cycles, start_point, end_point, collect_to_cycle_times, dim):
    matrix = np.ones((dim, dim)) * INF

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
    
    matrix *= 100
    matrix = matrix.astype(int)
    print(f"Clusters: {clusters}")
    return matrix, clusters
    
    

 
def solve_gtsp_with_release_collect(points, cycles, start_point, end_point, collect_to_cycle_times, plot=False):
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
    mapping_to_points[graph_index+1] = (4000, 0)  # Dummy point
    graph_index += 2
    dim = graph_index

    print(f"Mapping to release:")
    pprint(mapping_to_release)
    print(f"Mapping to collect:")
    pprint(mapping_to_collect)

    distance_matrix, clusters = buildt_GTSP_matrix(mapping_to_release, mapping_to_collect, points, cycles, start_point, end_point, collect_to_cycle_times, dim)
    gtsp_input_path = os.path.abspath("GTSP_input.gtsp")
    gtsp_output_path = os.path.abspath("GTSP_output.txt")
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
    
    if plot:
        clusters_for_plot = []
        for cix, cycle in enumerate(cycles):
            clusters_for_plot.extend([cix]*len(cycle))

 
        cluster_colors = cm.get_cmap('tab20', len(clusters))
        fig, ax = plt.subplots(figsize=(8, 6))
        # print(f"Mapping to points: {mapping_to_points}")
        for i, point in enumerate(points):
            x,y = point
            print(x,y)
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
        ax.set_xlim(-100, 4000)
        ax.set_ylim(-100, 4000)
        ax.grid(True)
        ax.relim()
        ax.autoscale_view()
        ax.set_title("GTSP with Release & Collect (Full Tour incl. Dummy)")
        ax.set_aspect('equal')
        plt.savefig('asdf.png')
#        plt.show()
 
    return {
        "raw_tour": tour,
        "path": path,
        "gtsp_file": gtsp_input_path,
        "output_file": gtsp_output_path,
        "solver_time": solver_time,
        "total_cost": total_cost
    }
 
def compute_all_cycle_times(cycles, points):
    """
    Computes UAV times for all cycles and all possible collect points in each cycle.
    Adds fixed takeoff and landing time to each.
    Checks feasibility against UAV battery and UGV travel limits.
    Returns two lists:
        all_times_list: list of dicts mapping all collect_idx to UAV time
        feasible_times_list: list of dicts mapping only feasible collect_idx to UAV time
    """
    points = np.array(points)
 
    collect_points = []
    for i, cycle in enumerate(cycles):
        collect_to_cycle_time = {}
        #TODO first and last should be different. 
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


            #
            # # #
            # # # # NEW CONSTRAINT WILL CHECK ONLY THE MAX BATTERY
            if ugv_time > UAV_BATTERY_TIME:
                continue
            # if ugv_time > time_sec:
            #     continue
            # # #
            # #
            #


            collect_to_cycle_time[collect_idx] = time_sec

            #TODO
            #total_dist = toal dis - return_dist + last_uav_treavel_actual_dist


        collect_points.append(collect_to_cycle_time)
    
    print("Collect points:")
    pprint(collect_points)

    return collect_points
 
def plot_original_cycles(original_points, original_cycles, start_point=None, end_point=None):
    colors = cm.get_cmap('tab20', len(original_cycles))
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
    plt.savefig('asdf2.png')
  #  plt.show()
 
 
 
 
def generate_points(n, x_range=(0, SPACE_SIZE), y_range=(0, SPACE_SIZE), FIXED_Z=FIXED_Z, decimals=2, seed=None):
    if seed is not None:
        random.seed(seed)
    points = [
        (
            round(random.uniform(*x_range), decimals),
            round(random.uniform(*y_range), decimals),
            FIXED_Z
        )
        for _ in range(n)
    ]
    print(f"Generated points:")
    pprint({i: p for i, p in enumerate(points)})
    return points
 
def create_distance_matrix(points):
    num_points = len(points)
    distance_matrix = np.zeros((num_points, num_points))
 
    for i in range(num_points):
        for j in range(num_points):
            distance_matrix[i][j] = euclidean(points[i], points[j])
   
    return distance_matrix
 
 
def find_closest_point(points, reference_point):
    """Find the index of the closest point to a given reference point."""
    return min(range(len(points)), key=lambda i: euclidean(points[i][:2], reference_point))
 
 
def reorder_points(points, start_index, end_index):
    """Reorder points with the start and end points in the correct positions."""
    reordered_points = [points[start_index]] + [
        p for i, p in enumerate(points) if i != start_index and i != end_index
    ] + [points[end_index]]
    return reordered_points
def solve_tsp_with_fixed_start_end(points, start_point, end_point):
    # Step 1: Find the closest points to START_POINT and END_POINT
    start_index = find_closest_point(points, start_point)
    end_index = find_closest_point(points, end_point)
   
    # Step 2: Reorder the points with the closest points to start and end at the beginning and end
    reordered_points = reorder_points(points, start_index, end_index)
 
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
        # routing_enums_pb2.FirstSolutionStrategy.CHRISTOFIDES)
        routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION)
 
    # COMMENT IF ONLY WANT TO USE THE FIRST SOL
    # Set local search metaheuristic to GUIDED_LOCAL_SEARCH
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
 
        plt.figure()
        if tsp_air_points:
            # Extract x,y (uses first two coordinates if points are 3D)
            xs = [p[0] for p in tsp_air_points]
            ys = [p[1] for p in tsp_air_points]
 
            # Plot the tour path with points
            plt.plot(xs, ys, marker='o')
 
            # Mark start and end (first and last in the tour order)
            plt.scatter(xs[0], ys[0], marker='s', s=100, label='Start (closest)')
            plt.scatter(xs[-1], ys[-1], marker='X', s=120, label='End (closest)')
 
 
            plt.title('TSP Tour with Fixed Start/End (reordered points)')
            plt.axis('equal')
            plt.grid(True)
            plt.legend()
        else:
            # If no solution, just scatter the reordered points for reference
            xs = [p[0] for p in reordered_points]
            ys = [p[1] for p in reordered_points]
            plt.scatter(xs, ys, marker='o')
            plt.title('Reordered Points (no tour found)')
            plt.axis('equal')
            plt.grid(True)
 
        plt.savefig('asdf3.png')
        # plt.show()
    # exit()
    return tsp_air_points

def create_cycles_CAHIT(tsp_tour):
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

            # Find best collect point for closing
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
    best_return_time = float('inf')
    best_uav_time = -float('inf')
    best_j = None
    for j in current_cycle:
        candidate = tsp_tour[j]
        return_time = euclidean(prev_point, candidate) / UAV_SPEED
        ugv_time = euclidean(cycle_start_point, candidate) / UGV_SPEED
        uav_time = current_time + return_time

        if uav_time <= UAV_BATTERY_TIME and ugv_time <= UAV_BATTERY_TIME:
            if uav_time > best_uav_time:
                best_uav_time = uav_time
                best_return_time = return_time
                best_j = j

    print(f"--> Closing at best collect {best_j}, return_time={best_return_time:.2f}")
    current_time += best_return_time
    cycles.append(current_cycle)
    cycle_times.append(current_time)
    print(f"Final cycle closed: {current_cycle}, total time={current_time:.2f}")

    print("\nCycles created CAHIT:")
    pprint(cycles)
    print(f"Cycle times CAHIT = {cycle_times}")
    return cycles





def run_experiments():
    filename = "our_cr2.txt"
 
    # Open results file in append mode
    with open(filename, "a", encoding="utf-8") as file:
        file.write("UGV_Speed, Num_Targets, Seed, Mission_Time, TSP_Computational_Time, GLNS_Computational_Time, Second_TSP_Comput_Time\n")  # Header
        for UGV_SPEED in UGV_SPEEDS:
            for num_points in NS:
                for seed in SEEDS:
                    # Generate points
                    points = generate_points(num_points, seed=seed)
                   
                    tsp_start_time = time.perf_counter()
                    # Solve TSP
                    tsp_points = solve_tsp_with_fixed_start_end(
                        points, START_POINT, END_POINT
                    )
 
                    # cycles = create_cycles(tsp_points)
                    tsp_end_time = time.perf_counter()
 
                    cycles = create_cycles_CAHIT(tsp_points)

                    collect_to_cycle_times = compute_all_cycle_times(
                        cycles,
                        tsp_points
                    )
 
                    result = solve_gtsp_with_release_collect(
                        points=tsp_points,
                        cycles=cycles,
                        start_point=(0, 0),
                        end_point=END_POINT,
                        collect_to_cycle_times=collect_to_cycle_times,
                        plot=True
                    )
                   
                   
                    mission_time = result["total_cost"] / 100
                    print("Total mission time (s):", mission_time)
 
                    tsp_time = tsp_end_time - tsp_start_time
                    glns_time = result["solver_time"] or -1  # fallback in case it's None
                    cycle_tsp_total_time = -1  # or fill if you compute it elsewhere
 
                    file.write(f"{UGV_SPEED},{num_points},{seed},{mission_time:.2f},{tsp_time:.4f},{glns_time:.4f},{cycle_tsp_total_time:.4f}\n")
 
 
 
if __name__ == '__main__':
    run_experiments()