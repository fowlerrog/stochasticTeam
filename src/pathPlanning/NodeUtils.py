# python imports
import numpy as np
from random import uniform
from scipy.spatial.distance import euclidean
from pprint import pprint

def generatePoints(params):
    """Generates a list of tuple points, with a params dict"""

    if params['TYPE'] == 'Uniform':
         return generateUniformPoints(
              params['NUM_POINTS'],
              xRange=(0, params['SPACE_SIZE']),
              yRange=(0, params['SPACE_SIZE']),
              fixedZ=params['FIXED_Z']
         )

def generateUniformPoints(n, xRange=(0, 1), yRange=(0, 1), fixedZ=0, decimals=2):
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
    return createFunctionMatrix(points, euclidean)

def createFunctionMatrix(l, f):
	"""
	Returns a matrix such that
	m[i][j] = f(l[i], l[j])
	"""
	m = np.zeros((len(l),len(l)))
	for i in range(len(l)):
		for j in range(len(l)):
			m[i][j] = f(l[i],l[j])
	return m

def createSubmatrix(m, indices):
    """
    Returns submatrix S s.t.
    S[i][j] = m[indices[i]][indices[j]]
    """
    return createFunctionMatrix(indices, lambda o, p : m[o][p])

def findClosestPoint(points, referencePoint):
    """Find the index of the closest point to a given reference point."""
    return min(range(len(points)), key=lambda i: euclidean(points[i], referencePoint))

def reorderList(l, startIndex, endIndex):
    """
    Reorder points with the start and end points in the correct positions.
    Note this will duplicate start and end values if they are the same.
    """
    reorderedPoints = [l[startIndex]] + [
        p for i, p in enumerate(l) if i != startIndex and i != endIndex
    ] + [l[endIndex]]
    return reorderedPoints
