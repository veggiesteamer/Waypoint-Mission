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

def distribute_stretch(total_m, min_d, max_d):
    """
    Target leg length for a stretch of total_m meters that must be split into
    equal-length legs each within [min_d, max_d] if any integer split count
    achieves that; otherwise returns the closest feasible equal split. See
    build_drive_segments' stretch pre-pass for how this gets applied.

    Degenerate config (non-positive or inverted bounds) or a non-positive
    total_m falls back to a single leg spanning the whole stretch -- never
    raises, so a bad config value can't crash a mission build.

    Note: near total_m == k * max_d for some integer k, the required split
    count jumps from k to k+1 and the resulting leg length can undershoot
    min_d by a wide margin (e.g. total_m=12.0001, [8,12] -> n=2 -> ~6.0).
    This is an unavoidable consequence of equal-length splitting with an
    integer leg count, not a bug -- it only affects totals right at that
    boundary, not typical path lengths.
    """
    if total_m <= 0:
        return 0.0
    if min_d <= 0 or max_d <= 0 or max_d < min_d:
        return total_m
    avg_d = (min_d + max_d) / 2
    n_min = max(1, math.ceil(total_m / max_d))
    n_max = max(n_min, math.floor(total_m / min_d))
    n = min(max(round(total_m / avg_d), n_min), n_max)
    return total_m / n


def _incoming_outgoing_headings(path, map_resolution, window_m):
    """
    Heading vectors at each path vertex, estimated by walking window_m/2 meters
    backward and forward *along the existing path polyline* (following its
    actual vertices, never a straight-line shortcut off it) to get an incoming
    and an outgoing direction. Comparing these -- instead of two adjacent raw
    grid steps, which on an 8-connected planning grid are only ~0.2-0.3 m long
    -- avoids flagging single-pixel staircase alternation (A* alternating
    between a straight and a diagonal move to hug a non-axis-aligned line) as
    a turn. Since both points are literal vertices of the original, already
    obstacle-free path, this never risks cutting a corner over an obstacle --
    it only changes how far apart the two points are that get compared.

    Returns (incoming, outgoing, valid), all aligned to path vertices. valid[i]
    is False where a full window isn't available (near the path ends).
    """
    pts = np.array(path, dtype=float)
    n = len(pts)
    idx = np.arange(n)

    seg_vectors = np.diff(pts, axis=0)
    seg_lengths_m = np.linalg.norm(seg_vectors, axis=1) / map_resolution
    cum_dist_m = np.concatenate([[0.0], np.cumsum(seg_lengths_m)])

    half = window_m / 2.0
    back_idx = np.searchsorted(cum_dist_m, cum_dist_m - half, side="right") - 1
    fwd_idx = np.searchsorted(cum_dist_m, cum_dist_m + half, side="left")

    valid = (back_idx >= 0) & (fwd_idx < n) & (back_idx < idx) & (fwd_idx > idx)

    back_idx_c = np.clip(back_idx, 0, n - 1)
    fwd_idx_c = np.clip(fwd_idx, 0, n - 1)

    in_vec = pts[idx] - pts[back_idx_c]
    out_vec = pts[fwd_idx_c] - pts[idx]

    in_norm = np.linalg.norm(in_vec, axis=1)
    out_norm = np.linalg.norm(out_vec, axis=1)
    valid &= (in_norm > 0) & (out_norm > 0)

    incoming = np.zeros((n, 2))
    outgoing = np.zeros((n, 2))
    safe_in_norm = np.where(in_norm > 0, in_norm, 1)
    safe_out_norm = np.where(out_norm > 0, out_norm, 1)
    incoming[valid] = (in_vec / safe_in_norm[:, None])[valid]
    outgoing[valid] = (out_vec / safe_out_norm[:, None])[valid]

    return incoming, outgoing, valid


def turn_analysis(path, angle_threshold_deg=10, map_resolution=1, window_m=1.0):
    """
    map_resolution: pixels per meter of the occupancy grid the path was planned on.
    The A* path is in pixel/grid coordinates, so all distance metrics are converted
    to meters by dividing by map_resolution (angles are scale-invariant, no conversion needed).
    window_m: baseline distance (see _incoming_outgoing_headings) used to detect a
    turn -- keep this well above the raw grid step size (1/map_resolution) so
    single-pixel staircase moves aren't counted as turns.
    """
    pathpoints = np.array(path)
    amount_pathpoints = len(path)

    # Segment vectors
    vectors = np.diff(np.array(path), axis=0)
    distances = np.linalg.norm(vectors, axis=1) / map_resolution

    ### Direction changes
    incoming, outgoing, valid = _incoming_outgoing_headings(path, map_resolution, window_m)
    dots = np.clip(np.sum(incoming * outgoing, axis=1), -1, 1)
    angles = np.degrees(np.arccos(dots))

    change_indices = np.where(valid & (angles > angle_threshold_deg))[0].tolist()

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


def build_drive_segments(path, map_resolution, distance_range, angle_threshold_deg=10, window_m=1.0,
                          phase_boundary_idx=None, special_trigger_idxs=None,
                          subphase_windows=None, max_stretch_multiple=10,
                          stop_angle_threshold_deg=30, turn_stop_floor_fraction=0.3):
    """
    Split the path into drive legs (the distance the rover covers before
    stopping to take a measurement / downlink), and count how many turns
    (direction changes above angle_threshold_deg, measured over a window_m
    baseline -- see _incoming_outgoing_headings/turn_analysis) occur within each leg.

    The path is first partitioned into "stretches", each split into
    equal-length legs that best fit within that stretch's own regime's
    [min_d, max_d] range (see distribute_stretch) -- so spacing varies with
    the stretch's own length instead of landing on one fixed number, and
    only the stretch's own trailing remainder (if any) is ever left over,
    not the whole path's. A stretch is a maximal run of constant (phase,
    subphase) spacing regime, additionally closed early either (a) at a
    detected corner -- angle_threshold_deg's turn_at is used for turn-time
    costing (below) but is too sensitive for this: on a real route it flags
    a turn every ~1-2m on average, mostly small-angle staircase/simplification
    noise, not genuine navigational corners, so stretch-closing instead uses
    its own coarser stop_angle_threshold_deg (same window_m) -- once at least
    turn_stop_floor_fraction * min_d has already accumulated since the last
    close, or (b) once accumulated distance reaches max_stretch_multiple *
    max_d regardless of corners (a backstop for a long corner-free straight
    run). turn_stop_floor_fraction < 1.0 trades away the "every closed
    stretch is a feasible distribute_stretch total" guarantee a full min_d
    floor would give (see distribute_stretch's own docstring) -- some
    stretches will come out shorter than min_d when a real corner sits close
    to the previous stop, by design: a corner is meant to always be a stop,
    even a near one, rather than getting walked through to preserve
    min_d. distribute_stretch's fallback (one leg spanning the whole
    stretch) handles this without crashing either way. Without any
    stop_angle_threshold_deg-level corner in the whole path (e.g. a
    synthetic straight test path), only condition (b) can ever fire.

    distance_range: either a (min_d, max_d) tuple (used for every stop) or a
    {phase: (min_d, max_d)} dict for per-phase ranges (falls back to phase
    0's value for any phase not listed). A single path vector can span
    multiple stops either way -- it's chunked into as many stops as it
    covers.

    phase_boundary_idx: path-vertex index marking the end of mission phase 0
    (e.g. the index of the first waypoint, for a report split into "traverse
    to the first waypoint" vs. "everything after"). Each segment is tagged
    "phase": 0 if it finishes at or before this index, else 1. None (default)
    tags every segment "phase": 0.

    special_trigger_idxs: path-vertex indices (e.g. user-marked "special"
    waypoints) that force a richer sequence onto the next regular stop --
    same snap-forward semantics as phase_boundary_idx. Only ever tags a
    segment "special" if that segment is actually phase 1 -- a trigger
    index in phase 0 (or with phase splitting disabled) never fires.

    subphase_windows: optional list of {"key": str, "trigger_idxs": [...],
    "half_width_m": float, "min_distance": float, "max_distance": float} --
    each defines a region, by distance walked along the path (not stop
    count), around every vertex in trigger_idxs (e.g. a user-marked "GRAV
    Dense Center" waypoint, see waypoint_picker.pick_special_waypoints)
    where that window's own (min_distance, max_distance) range applies
    instead of the parent phase's -- i.e. stop spacing itself changes here,
    not just which measurements fire once stopped (that part is
    conops_planner.build_mission's subphase_params). A segment ending
    inside any window is tagged "subphase": that window's key (first
    match wins if windows overlap); outside every window, "subphase": None
    and normal phase-based step_m applies unchanged. Not phase-restricted
    here -- it's a plain distance window around each anchor, computed once
    up front from the path's own geometry (see cum_dist_m below),
    independent of how the path later gets chunked into stops.

    Returns a list of {"distance": meters, "turns": count, "phase": 0 or 1,
    "special": bool, "subphase": str or None}, one entry per leg, for
    feeding into conops_planner.build_mission.
    """
    pts = np.array(path, dtype=float)
    vectors = np.diff(pts, axis=0)
    seg_lengths = np.linalg.norm(vectors, axis=1) / map_resolution
    # cumulative distance walked (m) up to each path vertex -- cum_dist_m[k]
    # is the distance from the path's start to vertex k
    cum_dist_m = np.concatenate([[0.0], np.cumsum(seg_lengths)])

    incoming, outgoing, valid = _incoming_outgoing_headings(path, map_resolution, window_m)
    dots = np.clip(np.sum(incoming * outgoing, axis=1), -1, 1)
    angles = np.degrees(np.arccos(dots))

    turn_at = (valid & (angles > angle_threshold_deg))[:len(vectors)]
    # coarser than turn_at -- see this function's docstring for why
    # stretch-closing needs its own threshold instead of reusing turn_at
    stop_turn_at = (valid & (angles > stop_angle_threshold_deg))[:len(vectors)]

    def phase_at(end_vertex_idx):
        if phase_boundary_idx is None or end_vertex_idx <= phase_boundary_idx:
            return 0
        return 1

    subphase_windows = subphase_windows or []

    def subphase_at(end_vertex_idx):
        d = cum_dist_m[end_vertex_idx]
        for win in subphase_windows:
            if any(abs(d - cum_dist_m[anchor_idx]) <= win["half_width_m"]
                   for anchor_idx in win["trigger_idxs"]):
                return win["key"]
        return None

    def range_for(phase, subphase_key):
        if subphase_key is not None:
            win = next(w for w in subphase_windows if w["key"] == subphase_key)
            return win["min_distance"], win["max_distance"]
        if isinstance(distance_range, dict):
            return distance_range.get(phase, distance_range.get(0))
        return distance_range

    trigger_idxs = sorted(special_trigger_idxs) if special_trigger_idxs else []
    trigger_ptr = 0

    def consume_special_trigger(end_vertex_idx):
        # A trigger only ever fires on a genuinely phase-1 segment -- the
        # only special_sequence that exists to run is phase 1's (see
        # build_mission), so a segment that still resolves to phase 0 can
        # never be tagged special, no matter what. But a *pending* trigger
        # whose own vertex sits at/before the phase boundary (e.g. a special
        # waypoint placed right at the pit entrance, which is itself still
        # counted as the last phase-0 stop -- see phase_at) is NOT dropped
        # here: trigger_ptr is left untouched so it carries over and gets
        # claimed by the first later segment that *does* close in phase 1,
        # i.e. "trigger right after this waypoint is passed" rather than
        # silently losing the occurrence. Consumes at most one pending
        # trigger per call even if several qualify, so two special waypoints
        # sharing one oversized simplified-path vector (see this function's
        # chunking loop below) still get one stop each instead of collapsing
        # onto the same early chunk-close.
        nonlocal trigger_ptr
        if phase_at(end_vertex_idx) != 1:
            return False
        if trigger_ptr < len(trigger_idxs) and trigger_idxs[trigger_ptr] <= end_vertex_idx:
            trigger_ptr += 1
            return True
        return False

    # Pre-pass: partition vector indices into stretches, and compute each
    # stretch's own leg length up front via distribute_stretch -- so a
    # stretch's total gets divided evenly by its own geometry instead of
    # chopped into fixed-size pieces with one remainder tacked onto the
    # whole path's end. A stretch is a maximal run of constant (phase,
    # subphase) regime (this key is provably monotonic/non-flapping --
    # cum_dist_m is monotonic and both phase_at/subphase_at are step
    # functions of it, so a regime run can't fragment into tiny one-vector
    # pieces near a window edge), additionally closed early at a detected
    # corner (stop_turn_at, coarser than turn_at) once
    # turn_stop_floor_fraction * min_d has accumulated, or at
    # max_stretch_multiple * max_d regardless of corners -- see this
    # function's docstring.
    current_step_m_per_vector = [0.0] * len(seg_lengths)
    run_start = 0
    while run_start < len(seg_lengths):
        regime = (phase_at(run_start + 1), subphase_at(run_start + 1))
        min_d, max_d = range_for(*regime)
        cap_m = max_stretch_multiple * max_d
        floor_m = turn_stop_floor_fraction * min_d
        run_end = run_start
        run_dist = 0.0
        while run_end < len(seg_lengths) and (phase_at(run_end + 1), subphase_at(run_end + 1)) == regime:
            run_dist += seg_lengths[run_end]
            run_end += 1
            corner_here = run_end < len(seg_lengths) and stop_turn_at[run_end]
            if (corner_here and run_dist >= floor_m) or run_dist >= cap_m:
                break
        seg_len = distribute_stretch(run_dist, min_d, max_d) if run_dist > 0 else (min_d + max_d) / 2
        for i in range(run_start, run_end):
            current_step_m_per_vector[i] = seg_len
        run_start = run_end

    segments = []
    acc_dist = 0.0
    acc_turns = 0

    for i, length in enumerate(seg_lengths):
        remaining = length
        # turn_at[i] flags vertex i, where vector i *starts* -- add it to the
        # pool as soon as this vector begins, same as the un-chunked case
        # below; it drains into whichever segment closes next
        if turn_at[i]:
            acc_turns += 1

        # a vector's phase/subphase can't change partway through --
        # phase_boundary_idx and every window boundary are always exact
        # path vertices -- so the threshold for this whole vector is fixed
        # up front
        end_subphase = subphase_at(i + 1)
        current_step_m = current_step_m_per_vector[i]

        # a single vector can be much longer than current_step_m (e.g. a long,
        # obstacle-free straight run left intact by simplify_path_line_of_sight)
        # -- chunk it into as many stops as it spans, instead of letting one
        # long vector become one oversized segment
        while acc_dist + remaining >= current_step_m:
            take = current_step_m - acc_dist
            remaining -= take
            acc_dist = 0.0
            segments.append({"distance": current_step_m, "turns": acc_turns, "phase": phase_at(i + 1),
                              "special": consume_special_trigger(i + 1), "subphase": end_subphase})
            acc_turns = 0

        acc_dist += remaining

    if acc_dist > 0:
        end_subphase = subphase_at(len(seg_lengths))
        segments.append({"distance": acc_dist, "turns": acc_turns, "phase": phase_at(len(seg_lengths)),
                          "special": consume_special_trigger(len(seg_lengths)), "subphase": end_subphase})

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


def _line_is_clear(grid, r0, c0, r1, c1):
    """
    Whether the straight line from (r0,c0) to (r1,c1) stays entirely in free
    cells (grid == 0). Oversamples 2x per cell along the line's dominant axis
    so no crossed cell is skipped.
    """
    dr, dc = r1 - r0, c1 - c0
    length_cells = max(abs(dr), abs(dc))
    if length_cells == 0:
        return grid[int(round(r0)), int(round(c0))] == 0

    num_samples = int(length_cells) * 2 + 1
    ts = np.linspace(0.0, 1.0, num_samples)
    rows = np.clip(np.round(r0 + dr * ts).astype(int), 0, grid.shape[0] - 1)
    cols = np.clip(np.round(c0 + dc * ts).astype(int), 0, grid.shape[1] - 1)
    return bool(np.all(grid[rows, cols] == 0))


def _simplify_segment_line_of_sight(path, grid):
    """
    Greedily reduce path to the fewest straight segments such that every
    kept segment's straight line stays entirely in free cells of grid --
    the same occupancy grid the path was planned on, so the simplified path
    can never cut through an obstacle border the raw A* path avoided.

    For each anchor point, binary-searches the farthest path index with a
    verified-clear line of sight and jumps straight there; safety never
    depends on the search finding the true global farthest point (every
    kept point is independently re-verified), only how much the path gets
    simplified does -- a correctness/optimality tradeoff, not a safety one.
    Runs in O(n log n) rather than the O(n^2) a naive point-by-point
    extension would risk on long, mostly-open paths.
    """
    if len(path) <= 2:
        return list(path)

    simplified = [path[0]]
    anchor_idx = 0
    n = len(path)

    while anchor_idx < n - 1:
        r0, c0 = path[anchor_idx]
        lo, hi = anchor_idx + 1, n - 1
        farthest = anchor_idx + 1  # the very next raw-path point is always reachable

        while lo <= hi:
            mid = (lo + hi) // 2
            r1, c1 = path[mid]
            if _line_is_clear(grid, r0, c0, r1, c1):
                farthest = mid
                lo = mid + 1
            else:
                hi = mid - 1

        simplified.append(path[farthest])
        anchor_idx = farthest

    return simplified


def simplify_path_line_of_sight(path, grid, pinned_points=None):
    """
    Line-of-sight path simplification (see _simplify_segment_line_of_sight)
    that removes the grid-staircase zigzag from an A* path while never
    letting a simplified segment cross an obstacle cell.

    pinned_points: optional ordered list of points from `path` (e.g. the
    mission waypoints a multi-leg route was required to pass through) that
    must remain exact vertices of the simplified output -- simplification
    is run independently between each consecutive pair of pinned points, so
    it can never skip over one even if a straight shortcut past it would
    otherwise be clear.
    """
    path = list(path)
    if pinned_points is None or len(pinned_points) <= 2:
        return _simplify_segment_line_of_sight(path, grid)

    pinned_indices = [0]
    search_from = 0
    for point in pinned_points[1:]:
        idx = path.index(tuple(point), search_from)
        pinned_indices.append(idx)
        search_from = idx

    simplified = [path[0]]
    for start_idx, end_idx in zip(pinned_indices[:-1], pinned_indices[1:]):
        leg_simplified = _simplify_segment_line_of_sight(path[start_idx:end_idx + 1], grid)
        simplified.extend(leg_simplified[1:])  # drop duplicate junction point

    return simplified