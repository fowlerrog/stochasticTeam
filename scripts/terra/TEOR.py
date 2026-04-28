import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Voronoi, voronoi_plot_2d
from matplotlib.patches import Circle
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import time
import heapq
from pprint import pprint
from tqdm import tqdm
from math import *

# ---------------------------
# Constants
# ---------------------------
TAKEOFF_LANDING_PENALTY = 100  # seconds
SPACE_SIZE = 4000              # working space dimensions
CHARGE_RATE = 1

# --- Stochastic travel time model: t ~ l * clamp(N(mu, sigma), mu-3s, mu+3s) ---
UAV_MU    = 0.1    # s/m
UAV_SIGMA = 0.01   # s/m  (scaled by sqrt(l) — see travel_time())
UGV_MU    = 0.4    # s/m
UGV_SIGMA = 0.04   # s/m

# Nominal speeds (used only for R conversion and budget mapping)
UAV_SPEED = 1.0 / UAV_MU   # = 10 m/s
UGV_SPEED = 1.0 / UGV_MU   # = 2.5 m/s

TAKEOFF_LANDING_DISTANCE = TAKEOFF_LANDING_PENALTY * UAV_SPEED  # 1000m

def travel_time(length, mu, sigma, rng=None):
    if rng is None:
        rng = np.random.default_rng()
    mean  = length * mu
    std   = sigma * np.sqrt(length)
    lo    = max(0.0, length * (mu - 3 * sigma))
    hi    = length * (mu + 3 * sigma)
    return float(np.clip(rng.normal(mean, std), lo, hi))

# ---------------------------
# Benchmark Budget Parameters
# ---------------------------
MU    = 600.0
SIGMA = 30.0

FLIGHT_BUDGET     = MU - TAKEOFF_LANDING_PENALTY
# TERRA_R           = (FLIGHT_BUDGET * UAV_SPEED)/2
TERRA_R = 2500
# TERRA_R_STOCH     = ((MU - sqrt(3)*SIGMA - TAKEOFF_LANDING_PENALTY) * UAV_SPEED)/2
TERRA_R_STOCH = 2131
# ---------------------------
# STEP 1: UGV Stop Selection via Voronoi and Coverage
# ---------------------------
def generate_target_points(n=5, xlim=(0, SPACE_SIZE), ylim=(0, SPACE_SIZE), seed=42):
    np.random.seed(seed)
    x = np.random.uniform(xlim[0], xlim[1], n)
    y = np.random.uniform(ylim[0], ylim[1], n)
    return np.column_stack((x, y))

def compute_voronoi(points):
    return Voronoi(points)

def filter_valid_vertices(voronoi, targets, R):
    valid_vertices = []
    for vertex in voronoi.vertices:
        if np.any([np.linalg.norm(vertex - t) < R for t in targets]):
            valid_vertices.append(vertex)
    return np.array(valid_vertices)

def assign_targets_to_vertices(targets, valid_vertices, R):
    nearest_vertices = {}
    uncovered = []
    for t in targets:
        if len(valid_vertices) == 0:
            uncovered.append(t)
            continue
        dists = np.linalg.norm(valid_vertices - t, axis=1)
        min_idx = np.argmin(dists)
        if dists[min_idx] <= R:
            nearest_vertices[tuple(t)] = tuple(valid_vertices[min_idx])
        else:
            uncovered.append(t)
    return nearest_vertices, uncovered

def compute_coverage_areas(nearest_vertices):
    coverage = {}
    for t, v in nearest_vertices.items():
        if v not in coverage:
            coverage[v] = []
        coverage[v].append(t)
    return coverage

def compute_junction_point_between_targets(p1, p2, R):
    x1, y1 = p1
    x2, y2 = p2
    dx, dy = x2 - x1, y2 - y1
    A = dx**2 + dy**2
    B = 2 * (dx * (x1 - x2) + dy * (y1 - y2))
    C = (x1 - x2)**2 + (y1 - y2)**2 - R**2
    disc = B**2 - 4 * A * C
    if disc < 0:
        return None
    t1 = (-B + np.sqrt(disc)) / (2 * A)
    t2 = (-B - np.sqrt(disc)) / (2 * A)
    candidates = []
    for t in (t1, t2):
        if 0 <= t <= 1:
            X = x1 + t * dx
            Y = y1 + t * dy
            candidates.append((X, Y))
    return candidates[0] if candidates else None

def greedy_set_cover(coverage_areas, all_targets):
    uncovered = set(tuple(t) for t in all_targets)
    local_coverage = {v: set(coverage_areas[v]) for v in coverage_areas}
    chosen_vertices = []
    while uncovered:
        best_vertex = None
        best_cover = set()
        for v, covered_set in local_coverage.items():
            intersection = covered_set & uncovered
            if len(intersection) > len(best_cover):
                best_cover = intersection
                best_vertex = v
        if not best_vertex:
            print("WARNING: Some targets cannot be covered by any remaining vertex.")
            break
        chosen_vertices.append(best_vertex)
        uncovered -= best_cover
        del local_coverage[best_vertex]
    return chosen_vertices

def compute_total_ugv_distance(ugv_stops, home_base=(0, 0)):
    stops = [home_base] + list(ugv_stops) + [home_base]
    return sum(np.linalg.norm(np.array(stops[i]) - np.array(stops[i+1])) for i in range(len(stops) - 1))

def compute_best_gravity_point(targets, home_base):
    targets_arr = np.array(targets)
    return {
        "XC": np.mean(targets_arr, axis=0),
        "MC": np.median(targets_arr, axis=0),
        "HC": np.array(home_base)
    }

def adjust_stop_toward_gravity(v, G_p, R):
    v_arr, G_p_arr = np.array(v), np.array(G_p)
    direction = G_p_arr - v_arr
    dist = np.linalg.norm(direction)
    if dist <= R:
        return tuple(G_p_arr)
    return tuple(v_arr + (direction / dist) * R)

def refine_ugv_stops_gravity(chosen_vertices, coverage_map, R, home_base=(0, 0)):
    refined_vertices = chosen_vertices.copy()
    global_targets = [t for targets in coverage_map.values() for t in targets]
    gravity_points = compute_best_gravity_point(global_targets, home_base)
    best_global_distance = compute_total_ugv_distance(refined_vertices, home_base)
    best_global_vertices = refined_vertices.copy()
    for method, G_p in gravity_points.items():
        temp_vertices = refined_vertices.copy()
        for i, v in enumerate(refined_vertices):
            assigned_targets = coverage_map.get(v, [])
            if not assigned_targets:
                continue
            candidate = adjust_stop_toward_gravity(v, G_p, R)
            if all(np.linalg.norm(np.array(candidate) - np.array(t)) <= R for t in assigned_targets):
                temp_vertices[i] = candidate
        new_distance = compute_total_ugv_distance(temp_vertices, home_base)
        if new_distance < best_global_distance:
            best_global_distance = new_distance
            best_global_vertices = temp_vertices.copy()
    new_coverage_map = {gnd: [] for gnd in best_global_vertices}
    for target in global_targets:
        closest_ugv = min(best_global_vertices, key=lambda g: np.linalg.norm(np.array(g) - np.array(target)))
        if np.linalg.norm(np.array(closest_ugv) - np.array(target)) <= R:
            new_coverage_map[closest_ugv].append(target)
    coverage_map.clear()
    coverage_map.update(new_coverage_map)
    for gnd_point in coverage_map:
        coverage_map[gnd_point] = set(coverage_map[gnd_point])
    return best_global_vertices, coverage_map

def build_distance_matrix(points):
    n = len(points)
    dist_matrix = np.zeros((n, n))
    for i in range(n):
        for j in range(n):
            dist_matrix[i, j] = np.linalg.norm(np.array(points[i]) - np.array(points[j]))
    return dist_matrix

def solve_tsp_with_ortools(distance_matrix, points=None):
    n = len(distance_matrix)
    manager = pywrapcp.RoutingIndexManager(n, 1, 0)
    routing = pywrapcp.RoutingModel(manager)
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(distance_matrix[from_node, to_node])
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    solution = routing.SolveWithParameters(search_parameters)
    if solution:
        route = []
        index = routing.Start(0)
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        route.append(manager.IndexToNode(index))
        route_distance = 0.0
        for i in range(len(route) - 1):
            route_distance += distance_matrix[route[i], route[i+1]]
        return route, route_distance
    else:
        print("No solution found by OR-Tools.")
        return None, None

def ugv_optimization(n_targets, R, seed, depot=(0,0)):
    target_points = generate_target_points(n=n_targets, seed=seed)
    vor = compute_voronoi(target_points)
    valid_vertices = filter_valid_vertices(vor, target_points, R)
    nearest_vertices, uncovered_targets = assign_targets_to_vertices(target_points, valid_vertices, R)
    dummy_point = None
    if len(uncovered_targets) == 2:
        p1, p2 = uncovered_targets
        dummy_point = compute_junction_point_between_targets(p1, p2, R)
        if dummy_point is not None:
            valid_vertices = np.vstack([valid_vertices, dummy_point])
            for t in uncovered_targets:
                if np.linalg.norm(np.array(dummy_point) - t) <= R:
                    nearest_vertices[tuple(t)] = tuple(dummy_point)
            uncovered_targets = [t for t in uncovered_targets if tuple(t) not in nearest_vertices]
    coverage_areas = compute_coverage_areas(nearest_vertices)
    coverage_areas = {v: set() for v in coverage_areas.keys()}
    for t in target_points:
        for v in coverage_areas.keys():
            if np.linalg.norm(np.array(v) - t) < R:
                coverage_areas[v].add(tuple(t))
    chosen_vertices = greedy_set_cover(coverage_areas, target_points)
    final_map = {}
    for v in chosen_vertices:
        final_map[v] = coverage_areas.get(v, set())
    refined_vertices, updated_coverage_map = refine_ugv_stops_gravity(chosen_vertices, final_map, R)
    stops = [depot] + list(refined_vertices)
    dist_matrix = build_distance_matrix(stops)
    route, route_cost = solve_tsp_with_ortools(dist_matrix)
    return refined_vertices, updated_coverage_map, stops, route, route_cost

# ---------------------------
# STEP 5: UAV Multi-Subtour A* Search
# ---------------------------
class UAVState:
    def __init__(self, pos, r, d=0.0, h=0.0, g=0.0, f=0.0, parent=None):
        self.pos = pos
        self.d = d
        self.h = h
        self.r = r
        self.f = f
        self.g = g
        self.parent = parent
    def __eq__(self, other):
        return (self.pos == other.pos) and (self.h == other.h) and (self.r == other.r)
    def path(self):
        s = str([int(self.pos[0]), int(self.pos[1])])
        parent = self.parent
        while parent is not None:
            s = str([int(parent.pos[0]), int(parent.pos[1])]) + "+" + s
            parent = parent.parent
        return s
    def __lt__(self, other):
        return self.f < other.f
    def __repr__(self):
        return f"h: {self.h} " + self.path()

def distance(a, b):
    return np.hypot(a[0] - b[0], a[1] - b[1])

def get_subtour(goal_state):
    path = []
    p_curr = goal_state
    while p_curr is not None:
        path.append(p_curr.pos)
        p_curr = p_curr.parent
    return path[::-1]

def in_path(n, p_curr):
    parent = p_curr.parent
    while parent is not None:
        if n == parent.pos:
            return True
        parent = parent.parent
    return False

def expand_graph(p_curr, p_start, R, a_v):
    p_neighbors = []
    for n in a_v:
        if p_curr.pos == n:
            continue
        if n == p_start.pos or (not in_path(n, p_curr)):
            d = distance(p_curr.pos, n) if p_curr.pos == p_start.pos else p_curr.d + distance(p_curr.pos, n)
            d_start = distance(p_start.pos, n)
            if 2 * R >= d + d_start:
                r = p_curr.r - 1 if n != p_start.pos else p_curr.r
                p_nbr = UAVState(pos=n, d=d, h=r, r=r, g=p_curr.g + distance(p_curr.pos, n), parent=p_curr)
                p_nbr.f = p_nbr.g + p_nbr.h
                p_neighbors.append(p_nbr)
    return p_neighbors

def update_node(open_heap, p_curr, p_nbr, p_start):
    d = distance(p_curr.pos, p_nbr.pos)
    if p_curr.g + d < p_nbr.g:
        p_nbr.g = p_curr.g + d
        p_nbr.parent = p_curr
        if p_nbr in open_heap:
            open_heap.remove(p_nbr)
            heapq.heapify(open_heap)
        p_nbr.f = p_nbr.g + p_nbr.h
        heapq.heappush(open_heap, p_nbr)
    return open_heap

def a_star_uav_multi_subtour(v, a_v, R, debug=False):
    a_v = a_v.copy()
    if v not in a_v:
        a_v.append(v)
    p_start = UAVState(v, r=len(a_v)-1, h=len(a_v)-1, f=len(a_v)-1)
    p_start.parent = None
    open_heap = []
    closed_list = []
    heapq.heappush(open_heap, p_start)
    while open_heap:
        p_curr = heapq.heappop(open_heap)
        if p_curr.pos == v and p_curr.r == 0:
            return get_subtour(p_curr), p_curr.g
        closed_list.append(p_curr)
        p_neighbors = expand_graph(p_curr, p_start, R, a_v)
        for p_nbr in p_neighbors:
            if p_nbr not in closed_list:
                if p_nbr not in open_heap:
                    p_nbr.g = float('inf')
                    p_nbr.parent = None
                else:
                    p_nbr = open_heap[open_heap.index(p_nbr)]
                open_heap = update_node(open_heap, p_curr, p_nbr, p_start)
    raise Exception("A* search did not find a solution.")

# ---------------------------
# Stochastic execution helpers
# ---------------------------
def check_battery(route_uav, stop, R, rng=None):
    if rng is None:
        rng = np.random.default_rng()
    battery_budget = 2 * R * UAV_MU
    accumulated = 0.0
    for j in range(1, len(route_uav)):
        seg = distance(route_uav[j - 1], route_uav[j])
        if route_uav[j - 1] == stop:
            accumulated = travel_time(seg, UAV_MU, UAV_SIGMA, rng)
        else:
            accumulated += travel_time(seg, UAV_MU, UAV_SIGMA, rng)
        time_to_stop = travel_time(distance(route_uav[j], stop), UAV_MU, UAV_SIGMA, rng)
        if accumulated + time_to_stop > battery_budget + 1e-6:
            return False, (f"Battery exceeded at leg {j-1}→{j}: "
                           f"used={accumulated:.1f}s + return={time_to_stop:.1f}s "
                           f"> budget={battery_budget:.1f}s")
    return True, ""

def post_process_uav_subtour(S, rng=None):
    if rng is None:
        rng = np.random.default_rng()
    total_flight_time   = 0.0
    total_charging_time = 0.0
    for route_uav in S:
        cycle_time = 0.0
        for j in range(1, len(route_uav)):
            seg = distance(route_uav[j - 1], route_uav[j])
            cycle_time += travel_time(seg, UAV_MU, UAV_SIGMA, rng)
            if route_uav[j] == route_uav[0]:
                cycle_time += travel_time(TAKEOFF_LANDING_DISTANCE, UAV_MU, UAV_SIGMA, rng)
                total_flight_time   += cycle_time
                total_charging_time += cycle_time * CHARGE_RATE
                cycle_time = 0.0
    return total_flight_time + total_charging_time

def stochastic_walk_ugv(route_cost, rng):
    """
    Walk the fixed UGV route stochastically.
    route_cost is the deterministic total UGV distance (metres).
    We sample a travel time for the whole route as one segment
    (or you could loop over individual legs if you store them).
    """
    return travel_time(route_cost, UGV_MU, UGV_SIGMA, rng)


def execute_plan_stochastic(S, ugv_stops, route_cost, R, rng):
    """
    Execute a fixed plan (UAV subtours S, UGV route distance route_cost)
    with one stochastic draw.  Returns (mission_time, ugv_time, uav_time, battery_ok, reason).
    """
    # UAV battery check
    for i, route_uav in enumerate(S):
        passed, reason = check_battery(route_uav, ugv_stops[i], R, rng)
        if not passed:
            return None, None, None, False, reason

    uav_time = post_process_uav_subtour(S, rng)
    ugv_time = stochastic_walk_ugv(route_cost, rng)
    return ugv_time + uav_time, ugv_time, uav_time, True, ""


# ---------------------------
# Plan builder (deterministic, called once per scenario)
# ---------------------------
def build_plan(n_targets, R, seed):
    """
    Build the deterministic plan: UGV stops + UAV subtours.
    Returns a dict with everything needed for stochastic execution.
    Raises on planning failure.
    """
    refined_vertices, updated_coverage_map, stops, route, route_cost = \
        ugv_optimization(n_targets, R, seed)

    ugv_stops   = list(updated_coverage_map.keys())
    target_sets = [list(val) for val in updated_coverage_map.values()]

    S = []
    for v, a_v in zip(ugv_stops, target_sets):
        route_uav, _ = a_star_uav_multi_subtour(v, a_v, R)
        S.append(route_uav)

    return {
        "ugv_stops":   ugv_stops,
        "target_sets": target_sets,
        "S":           S,                # list of UAV subtours (fixed geometry)
        "route_cost":  route_cost,       # deterministic UGV route length (m)
        "stops":       stops,
        "route":       route,
    }


# ---------------------------
# Benchmark runner
# ---------------------------
def run_benchmark(n_walks=100):
    NS    = [25, 50, 75, 100]
    SEEDS = [77, 0, 1, 2, 3, 4, 5, 6, 7, 8]   # 10 seeds
    OUTPUT   = "terra_benchmark.csv"
    FAIL_LOG = "terra_failures.txt"

    print(f"TERRA budget: R_stoch = (u + 3o - VTOL) x v = ({MU} + 3x{SIGMA} - {TAKEOFF_LANDING_PENALTY}) x {UAV_SPEED} = {TERRA_R_STOCH:.1f} m  (plan + execution)")

    with open(OUTPUT, "w", encoding="utf-8") as f, \
         open(FAIL_LOG, "w", encoding="utf-8") as flog:

        # CSV header: one row per (N, seed) aggregated over all walks
        f.write("Num_Targets,Seed,R,Avg_Mission_Time,Std_Mission_Time,"
                "Comp_Time,Fail_Rate,N_Walks\n")
        flog.write("TERRA Failure Log\n" + "="*40 + "\n")

        for n in NS:
            all_walk_times = []   # collect successful mission times across all seeds

            for seed in tqdm(SEEDS, desc=f"N={n}"):

                # ---- 1. Build plan ONCE (deterministic) -------------------------
                t_plan_start = time.perf_counter()
                try:
                    plan = build_plan(n, TERRA_R_STOCH, seed)
                except Exception as e:
                    reason = str(e)
                    flog.write(f"[PLAN_FAIL] N={n} seed={seed}: {reason}\n")
                    print(f"  PLAN_FAIL n={n} seed={seed}: {e}")
                    f.write(f"{n},{seed},{TERRA_R:.1f},NaN,NaN,NaN,1.0,0\n")
                    f.flush()
                    continue
                comp_time = time.perf_counter() - t_plan_start

                # ---- 2. Execute plan N_WALKS times stochastically ---------------
                seed_walk_times = []
                n_failures = 0

                for walk_id in range(n_walks):
                    walk_rng = np.random.default_rng(seed * 10_000 + walk_id)

                    mission_time, ugv_time, uav_time, battery_ok, reason = \
                        execute_plan_stochastic(
                            plan["S"],
                            plan["ugv_stops"],
                            plan["route_cost"],
                            TERRA_R,
                            walk_rng,
                        )

                    if not battery_ok:
                        n_failures += 1
                        flog.write(f"[BATTERY_FAIL] N={n} seed={seed} walk={walk_id}: {reason}\n")
                    else:
                        seed_walk_times.append(mission_time)

                # ---- 3. Aggregate over walks and write one row ------------------
                fail_rate = n_failures / n_walks
                if seed_walk_times:
                    avg_t = np.mean(seed_walk_times)
                    std_t = np.std(seed_walk_times)
                    all_walk_times.extend(seed_walk_times)
                else:
                    avg_t = std_t = float("nan")

                f.write(f"{n},{seed},{TERRA_R:.1f},{avg_t:.2f},{std_t:.2f},"
                        f"{comp_time:.4f},{fail_rate:.4f},{len(seed_walk_times)}\n")
                f.flush()

            if all_walk_times:
                print(f"  N={n} | successful walks={len(all_walk_times)}/{len(SEEDS)*n_walks} "
                      f"| mean={np.mean(all_walk_times):.1f}s  "
                      f"std={np.std(all_walk_times):.1f}s  "
                      f"median={np.median(all_walk_times):.1f}s  "
                      f"p95={np.percentile(all_walk_times, 95):.1f}s")

    print(f"\nResults saved to {OUTPUT}")


if __name__ == "__main__":
    run_benchmark(n_walks=1000)