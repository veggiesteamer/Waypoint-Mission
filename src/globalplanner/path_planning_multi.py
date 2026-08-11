from typing import List, Optional, Tuple

import numpy as np

from src.globalplanner import path_planning_astar
from src.globalplanner import path_planning_pyastar


def find_path_multi_waypoint(grid: np.ndarray, waypoints: List[Tuple[int, int]],
                              planner: str = "pyastar") -> Optional[List[Tuple[int, int]]]:
    """
    Plan a route through an ordered sequence of grid-coordinate waypoints
    (e.g. [start, wp1, wp2, ..., goal]) by chaining independent A* legs
    between consecutive waypoints and concatenating them into one path.
    Waypoints are visited in the given order, no reordering/optimization.

    Returns None if any leg fails to find a path.
    """
    if len(waypoints) < 2:
        raise ValueError("need at least 2 waypoints (start and goal)")

    if planner == "pyastar":
        find_path = path_planning_pyastar.find_path_astar
    elif planner == "astar":
        find_path = path_planning_astar.find_path_astar
    else:
        raise ValueError(f"Unknown planner {planner!r}, expected 'astar' or 'pyastar'")

    full_path = [tuple(waypoints[0])]

    for leg_start, leg_goal in zip(waypoints[:-1], waypoints[1:]):
        leg = find_path(grid, tuple(leg_start), tuple(leg_goal))
        if not leg:
            return None
        full_path.extend(leg[1:])  # drop duplicate junction point

    return full_path
