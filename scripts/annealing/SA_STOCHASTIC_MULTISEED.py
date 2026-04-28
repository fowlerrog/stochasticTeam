import random
import numpy as np
from simanneal import Annealer
from time import time
from scipy.stats import norm
import sys


# ===================== PARAMETERS =====================

SEEDS = [77, 0, 1, 2, 3, 4, 5, 6, 7, 8]
n = 10
SPACE_SIZE = 4000
FIXED_Z = 500

# Stochastic travel time: t = distance * N(mu_weight, sigma_weight)
UAV_WEIGHT = 0.1
UAV_WEIGHT_STD = 0.01
UGV_WEIGHT = 0.4
UGV_WEIGHT_STD = 0.04

battery_time = 600.0          # max UAV flight duration (tau_bar)
TAKEOFF_LANDING_TIME = 100.0  # fixed VTOL overhead per tour
CHARGE_RATE = 1               # recharge time = CHARGE_RATE * flight_time
FAILURE_RISK = 0.1            # p_r: acceptable probability of failure
LOG_SAFE_THRESHOLD = np.log(1 - FAILURE_RISK)  # log(0.9) ~ -0.10536

start_point = (0, 0, 0)
end_point = (4000, 4000, 0)


# ===================== POINT GENERATION =====================

def generate_uav_points(num, x_range=(0, 4000), y_range=(0, 4000), fixed_z=500, decimals=2, seed=None):
    if seed is not None:
        random.seed(seed)
    points = [
        (
            round(random.uniform(*x_range), decimals),
            round(random.uniform(*y_range), decimals),
            round(fixed_z, decimals)
        )
        for _ in range(num)
    ]
    print(points)
    return points

def generate_ugv_points(uav_pts):
    return [(x, y, 0) for x, y, _ in uav_pts]


# ===================== DISTANCE FUNCTIONS =====================

def euclidean_distance_2d(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def vtol_distance_3d(p1, p2):
    xy = euclidean_distance_2d(p1, p2)
    z = abs(p1[2] - p2[2])
    return xy + z


# ===================== STOCHASTIC TIME FUNCTIONS =====================

def uav_edge_mean(p1, p2):
    return vtol_distance_3d(p1, p2) * UAV_WEIGHT

def uav_edge_var(p1, p2):
    d = vtol_distance_3d(p1, p2)
    return (d ** 2) * (UAV_WEIGHT_STD ** 2)

def ugv_edge_mean(p1, p2):
    return euclidean_distance_2d(p1, p2) * UGV_WEIGHT

def ugv_edge_var(p1, p2):
    d = euclidean_distance_2d(p1, p2)
    return (d ** 2) * (UGV_WEIGHT_STD ** 2)


def tour_uav_mean_var(release, air_points, collect):
    mu = 0.0
    var = 0.0

    mu += uav_edge_mean(release, air_points[0])
    var += uav_edge_var(release, air_points[0])

    for j in range(len(air_points) - 1):
        mu += uav_edge_mean(air_points[j], air_points[j + 1])
        var += uav_edge_var(air_points[j], air_points[j + 1])

    mu += uav_edge_mean(air_points[-1], collect)
    var += uav_edge_var(air_points[-1], collect)

    mu += TAKEOFF_LANDING_TIME

    return mu, var


def log_prob_safe(mu, var, tau_bar):
    if var <= 0:
        if mu <= tau_bar:
            return 0.0
        else:
            return -1e10

    sigma = np.sqrt(var)
    z_val = (tau_bar - mu) / sigma

    if z_val > 8:
        return 0.0
    if z_val < -8:
        return -1e10

    return norm.logcdf(z_val)


def check_joint_chance_constraint(tour_stats, tau_bar):
    total_log_prob = 0.0

    for (mu_a, var_a, mu_g, var_g) in tour_stats:
        total_log_prob += log_prob_safe(mu_a, var_a, tau_bar)
        total_log_prob += log_prob_safe(mu_g, var_g, tau_bar)

    is_satisfied = (total_log_prob >= LOG_SAFE_THRESHOLD)
    return is_satisfied, total_log_prob


# ===================== SIMULATED ANNEALING =====================

class UAVUGVAnnealer(Annealer):
    def __init__(self, state, uav_points, start_point, end_point,
                 x_limit=(0, 4000), y_limit=(0, 4000), move_offset=4000):
        self.start_point = start_point
        self.end_point = end_point
        self.uav_points = uav_points
        self.x_limit = x_limit
        self.y_limit = y_limit
        self.move_offset = move_offset
        super(UAVUGVAnnealer, self).__init__(state)

    def temperature(self, step):
        return self.Tmax / (1 + np.log(step + 1))**0.5

    def energy(self):
        expected_total_time = 0.0

        expected_total_time += ugv_edge_mean(self.start_point, self.state[0, 0])

        tour_stats = []

        for i in range(n):
            release = self.state[i, 0]
            collect = self.state[i, -1]
            air_points = list(self.state[i, 1:-1])

            mu_a, var_a = tour_uav_mean_var(release, air_points, collect)

            mu_g = ugv_edge_mean(release, collect)
            var_g = ugv_edge_var(release, collect)

            tour_stats.append((mu_a, var_a, mu_g, var_g))

            expected_total_time += max(mu_a, mu_g)

            if i < n - 1:
                next_release = self.state[i + 1, 0]
                mu_g_transit = ugv_edge_mean(collect, next_release)
                recharge_time = CHARGE_RATE * mu_a
                expected_total_time += max(recharge_time, mu_g_transit)

        expected_total_time += ugv_edge_mean(self.state[-1, -1], self.end_point)

        is_safe, total_log_prob = check_joint_chance_constraint(
            tour_stats, battery_time
        )
        if not is_safe:
            violation = LOG_SAFE_THRESHOLD - total_log_prob
            expected_total_time += 50000 * (1 + violation)

        visited = set()
        for i in range(n):
            air_points = self.state[i, 1:-1]
            visited.update(air_points)

        missing = set(self.uav_points) - visited
        if missing:
            expected_total_time += 20000

        return expected_total_time

    def move(self):
        num_changes = np.random.randint(1, 4)

        for _ in range(num_changes):
            if np.random.rand() < 0.5:
                i = np.random.randint(0, n)
                self.move_ugv_point(i)
            else:
                row = np.random.randint(0, n)
                col = np.random.randint(1, n + 1)

                if np.random.rand() < 0.5:
                    self.state[row, col] = random.choice(self.uav_points)
                else:
                    self.state[row, col] = self.state[row, col - 1]

    def move_ugv_point(self, i):
        for col in [0, -1]:
            current = self.state[i, col]
            new_x = np.clip(
                current[0] + np.random.uniform(-self.move_offset, self.move_offset),
                self.x_limit[0], self.x_limit[1]
            )
            new_y = np.clip(
                current[1] + np.random.uniform(-self.move_offset, self.move_offset),
                self.y_limit[0], self.y_limit[1]
            )
            self.state[i, col] = (new_x, new_y, 0)


# ===================== POST-PROCESSING =====================

def analyze_best_solution(state, uav_points, start_point, end_point):
    total_time = 0.0
    n_rows = len(state)

    print("\n========== Analyzing Best Solution (Joint Probability) ==========")
    print("Failure risk p_r = %.4f, log(1-p_r) = %.6f" % (FAILURE_RISK, LOG_SAFE_THRESHOLD))

    mu_start = ugv_edge_mean(start_point, state[0, 0])
    total_time += mu_start
    print("UGV start -> first release: E[time] = %.2f" % mu_start)

    tour_stats = []

    for i in range(n_rows):
        release = state[i, 0]
        collect = state[i, -1]
        air_points = list(state[i, 1:-1])

        mu_a, var_a = tour_uav_mean_var(release, air_points, collect)
        sigma_a = np.sqrt(var_a) if var_a > 0 else 0.0

        mu_g = ugv_edge_mean(release, collect)
        var_g = ugv_edge_var(release, collect)
        sigma_g = np.sqrt(var_g) if var_g > 0 else 0.0

        tour_stats.append((mu_a, var_a, mu_g, var_g))

        lp_uav = log_prob_safe(mu_a, var_a, battery_time)
        lp_ugv = log_prob_safe(mu_g, var_g, battery_time)
        p_uav = np.exp(lp_uav) if lp_uav > -100 else 0.0
        p_ugv = np.exp(lp_ugv) if lp_ugv > -100 else 0.0

        print("\nTour %d:" % (i + 1))
        print("  Release: %s" % str(release))
        print("  Collect: %s" % str(collect))
        print("  UAV flight -- E=%.2f, std=%.2f, P(safe)=%.6f, logP=%.6f"
              % (mu_a, sigma_a, p_uav, lp_uav))
        print("  UGV catch  -- E=%.2f, std=%.2f, P(safe)=%.6f, logP=%.6f"
              % (mu_g, sigma_g, p_ugv, lp_ugv))
        print("  Battery limit: %.2f" % battery_time)

        tour_contribution = max(mu_a, mu_g)
        total_time += tour_contribution
        print("  Tour time contribution: max(E_uav, E_ugv) = %.2f" % tour_contribution)

        if i < n_rows - 1:
            next_release = state[i + 1, 0]
            mu_g_transit = ugv_edge_mean(collect, next_release)
            recharge_time = CHARGE_RATE * mu_a
            inter_tour = max(recharge_time, mu_g_transit)
            total_time += inter_tour
            print("  Inter-tour: max(recharge=%.2f, ugv_transit=%.2f) = %.2f"
                  % (recharge_time, mu_g_transit, inter_tour))

    mu_end = ugv_edge_mean(state[-1, -1], end_point)
    total_time += mu_end
    print("\nUGV last collect -> end: E[time] = %.2f" % mu_end)

    is_safe, total_log_prob = check_joint_chance_constraint(tour_stats, battery_time)
    joint_prob_safe = np.exp(total_log_prob) if total_log_prob > -100 else 0.0
    print("\n--- Joint Chance Constraint ---")
    print("  sum(log P(safe)) = %.6f" % total_log_prob)
    print("  log(1 - p_r)     = %.6f" % LOG_SAFE_THRESHOLD)
    print("  P(all safe)      = %.6f" % joint_prob_safe)
    print("  Required         >= %.6f" % (1 - FAILURE_RISK))
    print("  Satisfied: %s" % is_safe)

    if not is_safe:
        violation = LOG_SAFE_THRESHOLD - total_log_prob
        penalty = 50000 * (1 + violation)
        total_time += penalty
        print("  Penalty applied: %.2f" % penalty)

    visited = set()
    for i in range(n_rows):
        visited.update(state[i, 1:-1])
    missing = set(uav_points) - visited
    if missing:
        total_time += 20000
        print("\nWARNING: %d UAV points missed! Penalty applied." % len(missing))
    else:
        print("\nAll UAV points visited.")

    print("\nTotal Expected Mission Time (with penalties): %.2f" % total_time)
    return total_time


# ===================== MAIN EXECUTION =====================

orig_stdout = sys.stdout
f = open('stochastic_sa_results.txt', 'w')
sys.stdout = f

summary_file = "stochastic_sa_summary.txt"
with open(summary_file, "a") as summary:
    summary.write("SEED, STEPS, TMAX, BEST_ENERGY, COMPUTE_TIME\n")

STEPS = [50000, 100000, 150000]
TMAXS = [500.0, 1000.0, 1500.0]

REPEAT = 3
for seed in SEEDS:
    uav_points = generate_uav_points(n, seed=seed)
    ugv_points = generate_ugv_points(uav_points)

    for step in STEPS:
        for tmax in TMAXS:
            print("--------------o---------------- results for seed: %d, step: %d, tmax: %.1f" % (seed, step, tmax))

            # Initialize mission matrix
            mission_matrix = np.zeros((n, n + 2), dtype=tuple)
            for i in range(n):
                mission_matrix[i, 0] = ugv_points[i]
                for j in range(1, n + 1):
                    mission_matrix[i, j] = uav_points[i]
                mission_matrix[i, n + 1] = ugv_points[i]

            initial_state = mission_matrix.copy()
            annealer = UAVUGVAnnealer(initial_state, uav_points, start_point, end_point)
            annealer.steps = step
            annealer.Tmax = tmax
            annealer.Tmin = 0.01

            initial_energy = annealer.energy()
            print("\nInitial Expected Mission Time: %.2f" % initial_energy)

            start_time = time()
            best_state, best_energy = annealer.anneal()
            end_time = time()
            op_time = end_time - start_time

            print("\nInitial Mission Matrix:")
            print("START ->", start_point)
            for i in range(n):
                print(mission_matrix[i])
            print("END ->", end_point)

            print("\nBest Mission Matrix:")
            print("START ->", start_point)
            for i in range(n):
                print(best_state[i])
            print("END ->", end_point)
            print("\nBest Energy (Expected Mission Time): %.2f" % best_energy)
            print("OPTIMIZATION TOOK: %.2f seconds" % op_time)

            analyze_best_solution(best_state, uav_points, start_point, end_point)

            with open(summary_file, "a") as summary:
                summary.write("%d, %d, %.1f, %.2f, %.2f\n" % (seed, step, tmax, best_energy, op_time))