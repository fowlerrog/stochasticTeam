
from numpy import zeros
from random import uniform
from scipy.spatial.distance import euclidean
from pprint import pprint

def generatePoints(n, xRange=(0, 1), yRange=(0, 1), fixedZ=0, decimals=2):
    """
    Randomly generates n points in a rectangle xRange by yRange, with a fixed z coordinate
    n = int
    xRange, yRange = tuple
    fixedZ = float
    decimals = int
    returns a list of tuples
    """
    points = [
        (
            round(uniform(*xRange), decimals),
            round(uniform(*yRange), decimals),
            fixedZ
        )
        for _ in range(n)
    ]
    print(f"Generated points:")
    pprint({i: p for i, p in enumerate(points)})
    return points

def createDistanceMatrix(points):
    """Fills distance matrix for list of tuples"""
    numPoints = len(points)
    distanceMatrix = zeros((numPoints, numPoints))

    for i in range(numPoints):
        for j in range(numPoints):
            distanceMatrix[i][j] = euclidean(points[i], points[j])

    return distanceMatrix

def findClosestPoint(points, referencePoint):
    """Find the index of the closest point to a given reference point."""
    return min(range(len(points)), key=lambda i: euclidean(points[i], referencePoint))

def reorderList(l, startIndex, endIndex):
    """Reorder points with the start and end points in the correct positions."""
    reorderedPoints = [l[startIndex]] + [
        p for i, p in enumerate(l) if i != startIndex and i != endIndex
    ] + [l[endIndex]]
    return reorderedPoints
