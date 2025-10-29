
def escapePath(p):
    """Make paths safe for Julia calls"""
    return p.replace("\\", "\\\\")

def readGlnsOutput(path):
    """Reads a GLNS output file"""
    with open(path, "r") as f:
        for line in f:
            if "Tour" in line and "[" in line:
                tour = eval(line.split(":")[1].strip())
                tour = [int(x)-1 for x in tour]
            if "Solver Time" in line:
                solverTime = float(line.split(":")[1].strip().split()[0])
            if "Tour Cost" in line:
                totalCost = float(line.split(":")[1].strip())
    return tour, solverTime, totalCost

def writeGtspFile(index, clusters, distanceMatrix, gtspInputPath):
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
    for row in distanceMatrix:
        lines.append(" ".join(map(str, row)))
    lines.append("GTSP_SET_SECTION")
    for cid, cluster in enumerate(clusters, start=1):
        lines.append(f"{cid} " + " ".join(str(i + 1) for i in cluster) + " -1")
    lines.append("EOF")
    with open(gtspInputPath, "w") as f:
        f.write("\n".join(lines))

def tourToPath(tour, startIdx, dummyIdx):
    """Converts a tour (from GTSP output) to a path (list of indices)"""
    # TODO - this function just seems to reverse the tour
    startPos = tour.index(startIdx)
    dummyPos = tour.index(dummyIdx)

    if (startPos + 1) % len(tour) == dummyPos:
        path = tour[startPos+1:] + tour[:startPos+1]
        path = path[::-1]
    elif (startPos - 1) % len(tour) == dummyPos:
        path = tour[startPos:] + tour[:startPos]
    else:
        raise ValueError("startIdx and dummyIdx are not adjacent in the tour")

    # path = [x for x in path if x != dummyIdx]

    return path
