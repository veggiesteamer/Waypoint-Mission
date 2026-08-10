import numpy as np
import math

def analyze_path(path):

    pathpoints = np.array(path)

    # Segment vectors
    amount_pathpoints = len(path)
    vectors = np.diff(pathpoints, axis=0)

    distances = np.linalg.norm(vectors, axis=1)


    
    x_start,y_start = path[0]
    x_goal,y_goal = path[-1]
    distance_euclidean = np.sqrt((x_goal-x_start)**2+(y_goal-y_start)**2)

    distance_traverse = np.round(np.sum(distances),2)
    tortuosity = np.round(distance_traverse * 100 / distance_euclidean,2)

    return{
        "path_length": float(distance_traverse),
        "amount_pathpoints": int(amount_pathpoints),
        "tortuosity_factor": float(tortuosity)
    }


"""
    # Distances between waypoints
    distances = np.linalg.norm(vectors, axis=1)
    mean_distance = np.mean(distances)

    # Normalize vectors
    norms = np.linalg.norm(vectors, axis=1, keepdims=True)
    unit_vectors = vectors / norms
"""    
def turn_analysis(path, angle_threshold_deg=10, map_resolution=1):
    """
    map_resolution: pixels per meter of the occupancy grid the path was planned on.
    The A* path is in pixel/grid coordinates, so all distance metrics are converted
    to meters by dividing by map_resolution (angles are scale-invariant, no conversion needed).
    """
    pathpoints = np.array(path)
    amount_pathpoints = len(path)

    # Segment vectors
    vectors = np.diff(np.array(path), axis=0)
    distances = np.linalg.norm(vectors, axis=1) / map_resolution

    # Normalize vectors
    norms = np.linalg.norm(vectors, axis=1, keepdims=True)
    unit_vectors = vectors / norms

    ### Direction changes
    angles = []
    change_indices = []

    for i in range(len(unit_vectors)-1):
        dot = np.clip(np.dot(unit_vectors[i], unit_vectors[i+1]), -1, 1)
        angle = np.degrees(np.arccos(dot))
        angles.append(angle)

        if angle > angle_threshold_deg:
            change_indices.append(i+1)

    # --- Distances between direction changes ---
    segment_boundaries = [0] + change_indices + [len(pathpoints)-1]

    distances_between_turns = []

    for i in range(len(segment_boundaries)-1):
        start = segment_boundaries[i]
        end = segment_boundaries[i+1]

        segment_distance = np.sum(distances[start:end])
        distances_between_turns.append(segment_distance)

    mean_turn_distance = np.mean(distances_between_turns)


    x_start,y_start = path[0]
    x_goal,y_goal = path[-1]
    distance_euclidean = np.sqrt((x_goal-x_start)**2+(y_goal-y_start)**2) / map_resolution

    distance_traverse = np.round(np.sum(distances),2)
    tortuosity = np.round(distance_traverse * 100 / distance_euclidean,2)

    return{
        "path_length": float(distance_traverse),
        "amount_pathpoints": int(amount_pathpoints),
        "tortuosity_factor": float(tortuosity),
        "amount_turns": int(len(change_indices)),
        "mean_distance_before_turn": float(mean_turn_distance),
        "turns_per_100m": float(len(change_indices) * 100 / distance_traverse) if distance_traverse else 0.0
    }


def build_drive_segments(path, map_resolution, step_m, angle_threshold_deg=10):
    """
    Split the path into drive legs of ~step_m meters each (the distance the rover
    covers before stopping to take a measurement / downlink), and count how many
    turns (direction changes above angle_threshold_deg) occur within each leg.

    Returns a list of {"distance": meters, "turns": count}, one entry per leg,
    for feeding into conops_planner.build_mission.
    """
    pts = np.array(path, dtype=float)
    vectors = np.diff(pts, axis=0)
    seg_lengths = np.linalg.norm(vectors, axis=1) / map_resolution

    norms = np.linalg.norm(vectors, axis=1, keepdims=True)
    unit_vectors = vectors / norms

    turn_at = np.zeros(len(vectors), dtype=bool)
    for i in range(len(unit_vectors)-1):
        dot = np.clip(np.dot(unit_vectors[i], unit_vectors[i+1]), -1, 1)
        angle = np.degrees(np.arccos(dot))
        if angle > angle_threshold_deg:
            turn_at[i+1] = True

    segments = []
    acc_dist = 0.0
    acc_turns = 0

    for i, length in enumerate(seg_lengths):
        acc_dist += length
        if turn_at[i]:
            acc_turns += 1

        if acc_dist >= step_m:
            segments.append({"distance": acc_dist, "turns": acc_turns})
            acc_dist = 0.0
            acc_turns = 0

    if acc_dist > 0:
        segments.append({"distance": acc_dist, "turns": acc_turns})

    return segments


def analyze_path2(path, angle_threshold_deg=10):

    wp = np.array(path)

    # Segment vectors
    vectors = np.diff(wp, axis=0)

    # Distances between waypoints
    distances = np.linalg.norm(vectors, axis=1)
    mean_distance = np.mean(distances)

    # Normalize vectors
    norms = np.linalg.norm(vectors, axis=1, keepdims=True)
    unit_vectors = vectors / norms

    # Direction changes
    angles = []
    change_indices = []

    for i in range(len(unit_vectors)-1):
        dot = np.clip(np.dot(unit_vectors[i], unit_vectors[i+1]), -1, 1)
        angle = np.degrees(np.arccos(dot))
        angles.append(angle)

        if angle > angle_threshold_deg:
            change_indices.append(i+1)

    # --- Distances between direction changes ---
    segment_boundaries = [0] + change_indices + [len(wp)-1]

    distances_between_turns = []

    for i in range(len(segment_boundaries)-1):
        start = segment_boundaries[i]
        end = segment_boundaries[i+1]

        segment_distance = np.sum(distances[start:end])
        distances_between_turns.append(segment_distance)

    mean_turn_distance = np.mean(distances_between_turns)        

    print(mean_turn_distance)
    print(change_indices)
    print(angles)
    print(vectors)

    return mean_turn_distance, change_indices,angles,distances


def direction(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]

    # normalize to grid direction (-1,0,1)
    dx = 0 if dx == 0 else dx // abs(dx)
    dy = 0 if dy == 0 else dy // abs(dy)

    return (dx, dy)


def step_length(dir_vec):
    dx, dy = dir_vec
    if dx == 0 or dy == 0:
        return 10      # straight
    else:
        return 14      # diagonal


def compute_waypoints2(path):
    waypoints = [path[0]]

    current_dir = None
    distance_counter = 0

    for i in range(1, len(path)):
        p_prev = path[i-1]
        p_curr = path[i]

        dir_vec = direction(p_prev, p_curr)

        # detect direction change
        if current_dir is not None and dir_vec != current_dir:
            waypoints.append(p_prev)
            distance_counter = 0

        current_dir = dir_vec
        distance_counter += math.dist(p_prev, p_curr)

        if distance_counter >= step_length(current_dir):
            waypoints.append(p_curr)
            distance_counter = 0

    waypoints.append(path[-1])

    return waypoints

def compute_waypoints3(path,step):
    path = np.array(path, dtype=float)

    vectors = np.diff(path, axis=0)
    lengths = np.linalg.norm(vectors, axis=1)

    # normalized directions
    dirs = vectors / lengths[:, None]
    waypoints = [tuple(map(float, path[0]))]
    distances = [0.0]

    acc = 0
    dist_since_wp = 0
    current_dir = dirs[0]

    for i in range(len(vectors)):
        dist_since_wp += lengths[i]

        # direction change
        if not np.allclose(dirs[i], current_dir):
            waypoints.append(tuple(map(float, path[i])))

            acc = 0
            current_dir = dirs[i]
        distances.append(float(dist_since_wp))
        acc += lengths[i]

        if acc >= step - 1.42:
            waypoints.append(tuple(map(float, path[i+1])))
            acc = 0
            dist_since_wp = 0

    waypoints.append(tuple(map(float, path[-1])))
    distances.append(float(dist_since_wp))

    return waypoints, distances

def calculate_tortuosity(waypoints, distances):
    x_start,y_start = waypoints[0]
    x_goal,y_goal = waypoints[-1]
    distance_euclidean = np.sqrt((x_goal-x_start)**2+(y_goal-y_start)**2)
    distance_traverse = np.sum(distances)
    turtuosity = distance_traverse * 100 / distance_euclidean
    return turtuosity, distance_traverse, distance_euclidean
    

def compute_waypoints(path, step):
    path = np.array(path, dtype=float)
    
    waypoints = [tuple(map(float, path[0]))]
    distances = [0.0]

    acc = 0.0              # distance along current direction
    dist_since_wp = 0.0    # distance since last waypoint
    prev_dir = None

    for i in range(1, len(path)):
        vec = path[i] - path[i-1]
        dist = np.linalg.norm(vec)
        dir = tuple(np.sign(vec))  # approximate 8-grid direction

        # direction change or step reached
        if prev_dir is not None and (dir != prev_dir or acc + dist >= step):
            waypoints.append(tuple(map(float, path[i-1])))
            distances.append(float(dist_since_wp))
            acc = 0.0
            dist_since_wp = 0.0

        acc += dist
        dist_since_wp += dist
        prev_dir = dir

    # always add final point
    if waypoints[-1] != tuple(map(float, path[-1])):
        waypoints.append(tuple(map(float, path[-1])))
        distances.append(float(dist_since_wp))

    return waypoints, distances