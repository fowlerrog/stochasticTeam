
from numpy import zeros
from random import uniform
from scipy.spatial.distance import euclidean
from pprint import pprint

def generate_points(n, x_range=(0, 1), y_range=(0, 1), FIXED_Z=0, decimals=2):
    """
    Randomly generates n points in a rectangle x_range by y_range, with a fixed z coordinate
    n = int
    x_range, y_range = tuple
    fixed_z = float
    decimals = int
    returns a list of tuples
    """
    points = [
        (
            round(uniform(*x_range), decimals),
            round(uniform(*y_range), decimals),
            FIXED_Z
        )
        for _ in range(n)
    ]
    print(f"Generated points:")
    pprint({i: p for i, p in enumerate(points)})
    return points

def create_distance_matrix(points):
    """Fills distance matrix for list of tuples"""
    num_points = len(points)
    distance_matrix = zeros((num_points, num_points))

    for i in range(num_points):
        for j in range(num_points):
            distance_matrix[i][j] = euclidean(points[i], points[j])

    return distance_matrix

def find_closest_point(points, reference_point):
    """Find the index of the closest point to a given reference point."""
    return min(range(len(points)), key=lambda i: euclidean(points[i][:2], reference_point))

def reorder_list(l, start_index, end_index):
    """Reorder points with the start and end points in the correct positions."""
    reordered_points = [l[start_index]] + [
        p for i, p in enumerate(l) if i != start_index and i != end_index
    ] + [l[end_index]]
    return reordered_points
