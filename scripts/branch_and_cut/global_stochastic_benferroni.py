import gurobipy as gp
from gurobipy import GRB
import random
import numpy as np
from time import time
from scipy.stats import norm
import os


def generate_points(n, x_range=(0, 4000), y_range=(0, 4000),
                    fixed_z=500, decimals=2, seed=None):
    if seed is not None:
        random.seed(seed)
    points = [
        (round(random.uniform(*x_range), decimals),
         round(random.uniform(*y_range), decimals),
         round(fixed_z, decimals))
        for _ in range(n)
    ]
    print("Generated points: %s" % points)
    return points


def optimize_uav_ugv_route(points, max_cycles):

    N = len(points)

    # ===================== STOCHASTIC PARAMETERS =====================
    UAV_WEIGHT       = 0.1
    UAV_WEIGHT_STD   = 0.01
    UGV_WEIGHT       = 0.4
    UGV_WEIGHT_STD   = 0.04
    TAKEOFF_LANDING_TIME = 100.0
    MAX_FLIGHT_TIME      = 600.0
    CHARGE_RATE          = 1.0
    FAILURE_RISK         = 0.1

    # Bonferroni: split risk budget equally across 2*m terms (UAV+UGV per cycle)
    # Each term gets risk p_r / (2*m), so must succeed with prob >= 1 - p_r/(2m)
    # => z_alpha = Phi^{-1}(1 - p_r / (2*m))
    num_terms  = 2 * max_cycles
    z_alpha    = norm.ppf(1.0 - FAILURE_RISK / num_terms)
    print("Bonferroni z_alpha (per term): %.4f" % z_alpha)

    # ===================== TIGHT BIG-M VALUES =====================
    # Computed per-instance from actual point coordinates + grid bounds
    all_x = [p[0] for p in points] + [START[0], END[0]]
    all_y = [p[1] for p in points] + [START[1], END[1]]
    M_x = max(all_x) - min(all_x) + 1.0
    M_y = max(all_y) - min(all_y) + 1.0
    M_z = 500.0 + 1.0
    print("Tight M values: M_x=%.1f, M_y=%.1f, M_z=%.1f" % (M_x, M_y, M_z))

    model = gp.Model("UAV_UGV_Bonferroni")
    model.setParam("FeasibilityTol", 1e-4)
    model.setParam("NonConvex",      2)
    model.setParam("IntFeasTol",     1e-4)
    model.setParam("MipGap",         0.01)
    model.setParam("OptimalityTol",  1e-2)
    model.setParam("TimeLimit",      1200)

    # ===================== DECISION VARIABLES =====================
    x = model.addVars(max_cycles, N + 2, 3, lb=0, ub=4000,
                      vtype=GRB.CONTINUOUS, name="x")
    z = model.addVars(max_cycles, N, N, vtype=GRB.BINARY, name="z")
    model.update()

    # ===================== DISTANCE HELPER =====================
    def add_distance_var(name, diff0, diff1):
        d = model.addVar(vtype=GRB.CONTINUOUS, name=name, lb=0)
        model.addQConstr(d * d >= diff0 * diff0 + diff1 * diff1)
        return d

    # ===================== INDICATOR CONSTRAINTS (replaces Big-M) =====================
    # If z[cycle,visit-1,pt] == 1 then x[cycle,visit,:] == points[pt][:]
    # Gurobi handles branching logic internally — no M needed, tighter LP relaxation
    def add_indicator_constraints(model, x, z):
        for cycle in range(max_cycles):
            for visit in range(1, N + 1):
                for pt in range(N):
                    model.addGenConstrIndicator(
                        z[cycle, visit - 1, pt], True,
                        x[cycle, visit, 0] == points[pt][0],
                        name="ind_x_%d_%d_%d" % (cycle, visit, pt))
                    model.addGenConstrIndicator(
                        z[cycle, visit - 1, pt], True,
                        x[cycle, visit, 1] == points[pt][1],
                        name="ind_y_%d_%d_%d" % (cycle, visit, pt))
                    model.addGenConstrIndicator(
                        z[cycle, visit - 1, pt], True,
                        x[cycle, visit, 2] == points[pt][2],
                        name="ind_z_%d_%d_%d" % (cycle, visit, pt))
        return model

    # ===================== VISIT ALL POINTS =====================
    def add_visit_all_points_constraints(model, z):
        for pt in range(N):
            model.addConstr(
                gp.quicksum(
                    z[cycle, visit, pt]
                    for cycle in range(max_cycles)
                    for visit in range(N)
                ) >= 1, name="visit_%d" % pt)
        return model

    # ===================== REPEAT OR SAMPLE =====================
    def add_repeat_or_sample_constraint(model, x, z):
        for cycle in range(max_cycles):
            for visit in range(1, N + 1):
                sum_z = gp.quicksum(z[cycle, visit - 1, pt] for pt in range(N))
                for idx in range(3):
                    weighted = gp.quicksum(
                        z[cycle, visit - 1, pt] * points[pt][idx]
                        for pt in range(N))
                    model.addConstr(
                        x[cycle, visit, idx]
                        == weighted + (1 - sum_z) * x[cycle, visit - 1, idx],
                        name="rep_or_samp_%d_%d_%d" % (cycle, visit, idx))
        return model

    # ===================== DISTANCE VARIABLES =====================
    def add_distance_variables(model, x):
        vtol_dists, vtol_dists_sq = [], []

        for cycle in range(max_cycles):
            cycle_vtol, cycle_vtol_sq = [], []
            for visit in range(0, N + 1):
                # Altitude abs difference
                z_diff = model.addVar(lb=-501, ub=501,
                                      name="z_diff_%d_%d" % (cycle, visit))
                z_abs  = model.addVar(ub=501,
                                      name="z_abs_%d_%d" % (cycle, visit))
                model.addConstr(z_diff == x[cycle, visit, 2] - x[cycle, visit+1, 2])
                model.addGenConstrAbs(z_abs, z_diff)

                # XY Euclidean distance
                euc = add_distance_var(
                    "euc_%d_%d" % (cycle, visit),
                    x[cycle, visit, 0] - x[cycle, visit+1, 0],
                    x[cycle, visit, 1] - x[cycle, visit+1, 1])

                # VTOL = xy + |dz|
                vtol = model.addVar(name="vtol_%d_%d" % (cycle, visit))
                model.addConstr(vtol == euc + z_abs)
                cycle_vtol.append(vtol)

                # vtol² for variance
                vtol_sq = model.addVar(name="vtol_sq_%d_%d" % (cycle, visit))
                model.addQConstr(vtol_sq == vtol * vtol)
                cycle_vtol_sq.append(vtol_sq)

            vtol_dists.append(cycle_vtol)
            vtol_dists_sq.append(cycle_vtol_sq)

        # UGV release->collect
        ugv_cycle_dists, ugv_cycle_dists_sq = [], []
        for cycle in range(max_cycles):
            d = add_distance_var(
                "ugv_cyc_%d" % cycle,
                x[cycle, N+1, 0] - x[cycle, 0, 0],
                x[cycle, N+1, 1] - x[cycle, 0, 1])
            ugv_cycle_dists.append(d)
            dsq = model.addVar(name="ugv_cyc_sq_%d" % cycle)
            model.addQConstr(dsq == d * d)
            ugv_cycle_dists_sq.append(dsq)

        # UGV transit collect_i -> release_{i+1}
        ugv_transit_dists = []
        for cycle in range(max_cycles - 1):
            d = add_distance_var(
                "ugv_tr_%d" % cycle,
                x[cycle+1, 0, 0] - x[cycle, N+1, 0],
                x[cycle+1, 0, 1] - x[cycle, N+1, 1])
            ugv_transit_dists.append(d)

        dist_start = add_distance_var(
            "dist_start",
            x[0, 0, 0] - START[0],
            x[0, 0, 1] - START[1])

        dist_end = add_distance_var(
            "dist_end",
            x[max_cycles-1, N+1, 0] - END[0],
            x[max_cycles-1, N+1, 1] - END[1])

        return (vtol_dists, vtol_dists_sq, ugv_cycle_dists,
                ugv_cycle_dists_sq, ugv_transit_dists, dist_start, dist_end)

    # ===================== BONFERRONI CHANCE CONSTRAINT =====================
    # Risk budget split: each of 2*m terms must satisfy
    #   mu + z_alpha * sigma <= tau_bar
    # which is LINEAR once sigma is handled via:
    #   sigma^2 = sum(dist_sq * weight_std^2)  — still needs QConstr for std
    # This replaces the PWL entirely.
    def add_bonferroni_chance_constraint(model, vtol_dists, vtol_dists_sq,
                                          ugv_cycle_dists, ugv_cycle_dists_sq):
        for cycle in range(max_cycles):
            # --- UAV ---
            mu_a = model.addVar(lb=0, name="mu_a_%d" % cycle)
            model.addConstr(
                mu_a == gp.quicksum(vtol_dists[cycle]) * UAV_WEIGHT
                + TAKEOFF_LANDING_TIME)

            var_a = model.addVar(lb=0, name="var_a_%d" % cycle)
            model.addConstr(
                var_a == gp.quicksum(vtol_dists_sq[cycle]) * UAV_WEIGHT_STD**2)

            std_a = model.addVar(lb=1e-6, name="std_a_%d" % cycle)
            model.addQConstr(std_a * std_a == var_a)

            # Bonferroni: mu_a + z_alpha * std_a <= tau_bar
            model.addQConstr(
                mu_a + z_alpha * std_a <= MAX_FLIGHT_TIME,
                name="bonf_uav_%d" % cycle)

            # --- UGV ---
            mu_g = model.addVar(lb=0, name="mu_g_%d" % cycle)
            model.addConstr(mu_g == ugv_cycle_dists[cycle] * UGV_WEIGHT)

            var_g = model.addVar(lb=0, name="var_g_%d" % cycle)
            model.addConstr(
                var_g == ugv_cycle_dists_sq[cycle] * UGV_WEIGHT_STD**2)

            std_g = model.addVar(lb=1e-6, name="std_g_%d" % cycle)
            model.addQConstr(std_g * std_g == var_g)

            # Bonferroni: mu_g + z_alpha * std_g <= tau_bar
            model.addQConstr(
                mu_g + z_alpha * std_g <= MAX_FLIGHT_TIME,
                name="bonf_ugv_%d" % cycle)

        return model

    # ===================== GROUND CONSTRAINTS =====================
    def add_ugv_ground_constraints(model, x):
        for cycle in range(max_cycles):
            model.addConstr(x[cycle, 0,   2] == 0)
            model.addConstr(x[cycle, N+1, 2] == 0)
        return model

    # ===================== OBJECTIVE =====================
    def add_objective(model, vtol_dists, ugv_cycle_dists,
                      ugv_transit_dists, dist_start, dist_end):
        model.update()

        time_start = dist_start * UGV_WEIGHT
        time_end   = dist_end   * UGV_WEIGHT

        tour_times = []
        for cycle in range(max_cycles):
            mean_uav = model.addVar(name="mean_uav_%d" % cycle)
            model.addConstr(
                mean_uav == gp.quicksum(vtol_dists[cycle]) * UAV_WEIGHT
                + TAKEOFF_LANDING_TIME)

            mean_ugv = model.addVar(name="mean_ugv_%d" % cycle)
            model.addConstr(mean_ugv == ugv_cycle_dists[cycle] * UGV_WEIGHT)

            tt = model.addVar(name="tour_time_%d" % cycle)
            model.addGenConstrMax(tt, [mean_uav, mean_ugv],
                                  name="max_tour_%d" % cycle)
            tour_times.append(tt)

        inter_times = []
        for cycle in range(max_cycles - 1):
            # FIX: tau_c = gamma * max(tau_a, tau_g) = gamma * tour_time
            recharge = model.addVar(name="recharge_%d" % cycle)
            model.addConstr(recharge == CHARGE_RATE * tour_times[cycle])

            transit = model.addVar(name="transit_%d" % cycle)
            model.addConstr(transit == ugv_transit_dists[cycle] * UGV_WEIGHT)

            it = model.addVar(name="inter_time_%d" % cycle)
            model.addGenConstrMax(it, [recharge, transit],
                                  name="max_inter_%d" % cycle)
            inter_times.append(it)

        model.setObjective(
            time_start
            + gp.quicksum(tour_times)
            + gp.quicksum(inter_times)
            + time_end,
            GRB.MINIMIZE)

        return tour_times, inter_times, time_start, time_end

    # ===================== BUILD =====================
    model = add_indicator_constraints(model, x, z)
    model = add_visit_all_points_constraints(model, z)
    model = add_ugv_ground_constraints(model, x)
    model = add_repeat_or_sample_constraint(model, x, z)

    (vtol_dists, vtol_dists_sq, ugv_cycle_dists,
     ugv_cycle_dists_sq, ugv_transit_dists,
     dist_start, dist_end) = add_distance_variables(model, x)

    model = add_bonferroni_chance_constraint(
        model, vtol_dists, vtol_dists_sq,
        ugv_cycle_dists, ugv_cycle_dists_sq)

    tour_times, inter_times, time_start, time_end = add_objective(
        model, vtol_dists, ugv_cycle_dists,
        ugv_transit_dists, dist_start, dist_end)

    # ===================== SOLVE =====================
    model.optimize()

    if model.status not in (GRB.OPTIMAL, GRB.SUBOPTIMAL):
        print("No solution found. Status: %d" % model.status)
        try:
            model.computeIIS()
            ilp_path = os.path.abspath("SIKINTI.ilp")
            model.write(ilp_path)
            with open(ilp_path, "r") as fh:
                print(fh.read())
        except gp.GurobiError as e:
            print("IIS error: %s" % e)
        return None

    solution_x = model.getAttr("x", x)

    sampled_points = {}
    for cycle in range(max_cycles):
        sampled_points[cycle] = []
        for visit in range(1, N + 1):
            for pt in range(N):
                if z[cycle, visit - 1, pt].X > 0.5:
                    sampled_points[cycle].append(points[pt])

    cycle_times = []
    for cycle in range(max_cycles):
        ct = (sum(model.getVarByName("vtol_%d_%d" % (cycle, v)).X
                  for v in range(N + 1))
              * UAV_WEIGHT + TAKEOFF_LANDING_TIME)
        cycle_times.append(ct)

    transition_times = [time_start.getValue()]
    for cycle in range(max_cycles - 1):
        transition_times.append(
            model.getVarByName("inter_time_%d" % cycle).X)
    transition_times.append(time_end.getValue())

    return {
        "uav_points": [
            (solution_x[cycle, visit, 0],
             solution_x[cycle, visit, 1],
             solution_x[cycle, visit, 2])
            for cycle in range(max_cycles)
            for visit in range(1, N + 1)
        ],
        "ugv_points": [
            (solution_x[cycle, 0, 0],
             solution_x[cycle, 0, 1],
             solution_x[cycle, 0, 2])
            for cycle in range(max_cycles)
        ],
        "cycle_times":      cycle_times,
        "transition_times": transition_times,
        "sampled_points":   sampled_points,
        "objective":        model.ObjVal,
    }


# ===================== MAIN =====================

N_values = [2, 3, 4]
# seeds    = [77, 42, 12, 777, 888, 17, 21, 314, 2000, 93]
seeds    = [77, 0, 1, 2, 3, 4, 5, 6, 7, 8]

results_file = "results_bonferroni.txt"
points_file = "points.txt"

with open(results_file, "w") as f, open(points_file, "w") as f2:
    f2.write("N,seed,points")
    f.write("N,seed,mission_time,objective,computation_time\n")

    for N in N_values:
        for seed in seeds:
            points = generate_points(N, seed=seed)
            f2.write(f'{N}, {seed}, {points}\n')

            START  = [0, 0]
            END    = [4000, 4000]
            max_cycles = N

            t0      = time()
            results = optimize_uav_ugv_route(points, max_cycles)
            comp    = time() - t0

            if results:
                mt  = sum(results["cycle_times"]) + sum(results["transition_times"])
                obj = results["objective"]
                f.write("%d,%d,%.2f,%.2f,%.2f\n" % (N, seed, mt, obj, comp))
                f.flush()
                print("N:%d seed:%d -> Obj:%.2f Comp:%.2fs" % (N, seed, obj, comp))
            else:
                f.write("%d,%d,infeasible,NA,%.2f\n" % (N, seed, comp))
                f.flush()
                print("N:%d seed:%d -> Infeasible Comp:%.2fs" % (N, seed, comp))