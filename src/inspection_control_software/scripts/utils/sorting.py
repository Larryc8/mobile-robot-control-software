from ast import Dict
import math
import itertools
from typing import Any, List, Tuple

# Type alias for a point to make signatures cleaner
Point = Tuple[float, float]

def calculate_distance(p1: Point, p2: Point) -> float:
    """Calculates the Euclidean distance between two points."""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def func(p, current_point) -> None:
    id1: str
    point1: dict

    id2: str
    point2: dict

    id1, point1 = p 
    id2, point2 = current_point
    return  calculate_distance((point1.get('x_meters'), point1.get('y_meters')), (point2.get('x_meters'), point2.get('y_meters')))

def sort_nearest_neighbor(points: dict, current_position) -> None:
    """
    Finds a short path using the Nearest Neighbor heuristic.
    Very fast, but not guaranteed to be the optimal solution.
    """
    if not points:
        return {}
        
    unvisited: list = list(points.items() )# Make a copy
    path: list = []
    total_distance: float = 0.0
    
    # Start the path at the first point
    # current_point: Point = unvisited.pop(0)
    # path.append(current_point)

    print(path)
    print(unvisited)


    nearest_point = min(unvisited, key=lambda p: func(current_position, p))
    current_point = nearest_point
    # path.append(current_point)
    # unvisited.remove(current_point)
    
    while unvisited:
        nearest_point = min(unvisited, key=lambda p: func(current_point, p))

        total_distance += func(current_point, nearest_point)
        current_point = nearest_point
        path.append(current_point)
        unvisited.remove(current_point)

    print('path from sort sort_nearest_neighbor:', path)
    
    # Add distance from the last point back to the start to close the loop
    # total_distance += calculate_distance(path[-1], path[0])
    # path.append(path[0])
    path_dict_original = {key: value for key, value in path}
    path.reverse()
    path_dict = {key: value for key, value in path}

    return path_dict, path_dict_original
# --- Main execution ---

