import numpy as np
import pyastar2d
from typing import List, Tuple, Optional


def find_path_astar(grid: np.ndarray, start: Tuple[int, int],
                     goal: Tuple[int, int], allow_diagonal: bool = True) -> Optional[List[Tuple[int, int]]]:
    """
    Same interface as path_planning_astar.find_path_astar (0/1 occupancy grid
    in, path or None out), backed by the pyastar2d C-extension instead of a
    pure-Python heapq search. Orders of magnitude faster on large grids since
    the search itself runs compiled rather than as a Python loop.
    """
    weights = np.where(grid == 0, 1.0, np.inf).astype(np.float32)
    path = pyastar2d.astar_path(weights, start, goal, allow_diagonal=allow_diagonal)

    if path is None:
        return None

    return [tuple(p) for p in path.tolist()]
