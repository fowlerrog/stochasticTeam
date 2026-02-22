# This script is designed to receive a json from the VenvScriptCaller node, which will call it with the Python 3.9 .venv in the main repo. It performs one instance of online planning, then returns by printing.

if __name__ == '__main__':
    import sys
    import json
    from pathPlanning.PlannerUtils import plannerFromParams

    # Read JSON from stdin, assumed to be:
    # {
    # 'plannerParams': { contents of planner_params.yaml },
    # 'planParams': { contents of plan_path_results.yaml },
    # 'iTour': UAV tour index,
    # 'jTour': UAV index in tour i,
    # 'ugvIndex': UGV index in path,
    # 'uavPos': UAV position (3D),
    # 'ugvPos': UGV position (2D),
    # 'flightTime': current UAV flight time
    # }
    data = json.load(sys.stdin)

    # if any of this breaks, we just throw the error and let the requester handle it

    # Create online planner
    onlinePlanner = plannerFromParams(data['plannerParams'])

    # Solve
    thisUavTours, thisUgvOrder, thisUgvPoints, successFlag = onlinePlanner.solve(
        data['planParams']['uav_tours'], data['planParams']['uav_points'],
        data['planParams']['ugv_path'], data['planParams']['ugv_point_map'],
        data['iTour'], data['jTour'], data['ugvIndex'],
        data['uavPos'], data['ugvPos'],
        data['flightTime']
    )

    # Output result (captured by the node)
    print(json.dumps({
        'uavTours': thisUavTours,
        'ugvOrder': thisUgvOrder,
        'ugvPoints': thisUgvPoints,
        'successFlag': successFlag
    }))
