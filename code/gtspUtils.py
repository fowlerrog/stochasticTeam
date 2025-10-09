
def escape_path(p):
    """Make paths safe for Julia calls"""
    return p.replace("\\", "\\\\")

def read_glns_output(path):
    """Reads a GLNS output file"""
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
    """Writes a GTSP input file"""
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
    """Converts a tour (from GTSP output) to a path (list of indices)"""
    # TODO - this function just seems to reverse the tour
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
