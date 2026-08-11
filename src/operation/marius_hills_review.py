from src.globalplanner import path_planning_astar
from src.globalplanner import path_planning_pyastar
from src.mapdata import distribution_generator
from src.operation import path_analyzer
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import json
from pathlib import Path

R_MOON = 1737400


def _project_lonlat_to_local_m(lon, lat, lat0, lon0):
    x = np.radians(lon - lon0) * R_MOON * np.cos(np.radians(lat0))
    y = np.radians(lat - lat0) * R_MOON
    return x, y


def load_mhp_craters(csv_path):
    """
    Load classified Marius Hills Pit craters and project lon/lat to local
    meters, then normalize to [0, 1] so they line up with the synthetic
    (below-detection-threshold) crater/boulder layers, which are generated
    directly in normalized map coordinates.

    Also returns the projection frame (origin + normalization bounds), so
    other real-data layers (e.g. the pit outline) can be placed in the
    same map coordinate system.
    """
    df = pd.read_csv(csv_path)
    df.rename(columns={'x_coord': 'lon', 'y_coord': 'lat'}, inplace=True)

    lat0 = df['lat'].mean()
    lon0 = df['lon'].mean()

    df['x'], df['y'] = _project_lonlat_to_local_m(df['lon'], df['lat'], lat0, lon0)

    x_min, x_max = df['x'].min(), df['x'].max()
    y_min, y_max = df['y'].min(), df['y'].max()

    df['x_norm'] = (df['x'] - x_min) / (x_max - x_min)
    df['y_norm'] = (df['y'] - y_min) / (y_max - y_min)

    center = np.column_stack((df['x_norm'], df['y_norm']))
    radius = np.round(df['Diam_km'] * 1000 / 2, 4)

    frame = {"lat0": lat0, "lon0": lon0,
             "x_min": x_min, "x_max": x_max,
             "y_min": y_min, "y_max": y_max}

    return center, radius, frame


def load_mhp_pit_outline(path, frame):
    """
    Load the Marius Hills Pit boundary, stored as a single closed ring
    [[[lon, lat], ...]], and project/normalize it into the same map
    coordinate frame returned by load_mhp_craters, so the pit lines up
    spatially with the crater layer.
    """
    with open(path) as f:
        rings = json.loads(f.read())

    ring = np.array(rings[0])
    lon, lat = ring[:, 0], ring[:, 1]

    x, y = _project_lonlat_to_local_m(lon, lat, frame["lat0"], frame["lon0"])
    x_norm = (x - frame["x_min"]) / (frame["x_max"] - frame["x_min"])
    y_norm = (y - frame["y_min"]) / (frame["y_max"] - frame["y_min"])

    return np.column_stack((x_norm, y_norm))


def save_path_plot(grid, path, output_path):
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(grid, cmap='binary', origin='lower')

    if path:
        path_arr = np.array(path)
        ax.plot(path_arr[:, 1], path_arr[:, 0], 'b-', linewidth=3, label='Path')
        ax.plot(path_arr[0, 1], path_arr[0, 0], 'go', markersize=15, label='Start')
        ax.plot(path_arr[-1, 1], path_arr[-1, 0], 'ro', markersize=15, label='Goal')
        ax.legend(fontsize=12)

    ax.set_title("A* Pathfinding Result")
    ax.set_xlabel("Pixels")
    ax.set_ylabel("Pixels")

    fig.savefig(output_path, dpi=300)
    plt.close(fig)


def generate_map(config, dist_image_path="Marius_Hills_Mapped.png",
                  raster_image_path="Marius_Hills_Rasterized.png"):
    """
    Load hazards (real craters, the pit outline, synthetic craters/boulders),
    rasterize, and downsample -- the expensive, cacheable part of the
    pipeline (see load_or_generate_map). Does not plan any path.
    """
    map_distance_meter = config["map_size_m"]
    map_resolution = config["resolution"]
    robot_footprint = config["robot_footprint"]

    # crater production
    keep_out_crater = config["keep_out_crater"]
    k_crater = config["k_crater"]
    b_crater = config["b_crater"]
    Dmin_crater = config["Dmin_crater"]
    Dmax_crater = config["Dmax_crater"]

    # rock production
    keep_out_boulder = config["keep_out_boulder"]
    k_boulder = config["k_boulder"]
    b_boulder = config["b_boulder"]
    Dmin_boulder = config["Dmin_boulder"]
    Dmax_boulder = config["Dmax_boulder"]

    center_MHP, radius_MHP, mhp_frame = load_mhp_craters(config["csv_path"])
    pit_outline = load_mhp_pit_outline(config["pit_outline_path"], mhp_frame)

    center_crater, radius_crater = distribution_generator.center_radius_generator_distribution(
        map_distance_meter, map_resolution, keep_out_crater, robot_footprint,
        k_crater, b_crater, Dmin_crater, Dmax_crater)
    center_boulder, radius_boulder = distribution_generator.center_radius_generator_distribution(
        map_distance_meter, map_resolution, keep_out_boulder, robot_footprint,
        k_boulder, b_boulder, Dmin_boulder, Dmax_boulder)

    lunar_distribution_data = {
        "center": np.vstack([center_MHP, center_crater, center_boulder]),
        "radius": np.concatenate([radius_MHP, radius_crater, radius_boulder]),
        "type": np.array(["MHP_crater"] * len(radius_MHP)
                          + ["crater"] * len(radius_crater)
                          + ["boulder"] * len(radius_boulder))
    }

    ax = distribution_generator.distribution_vizualization_layers(
        map_distance_meter, map_resolution, lunar_distribution_data,
        show_MHP=True, show_craters=True, show_boulders=True,
        pit_outline=pit_outline, show_pit=True)
    fig = plt.gcf()
    fig.savefig(dist_image_path, dpi=300)
    plt.close(fig)

    grid_size = map_resolution * map_distance_meter
    grid = distribution_generator.rasterize_circles(
        lunar_distribution_data, map_distance_meter, grid_size)
    pit_grid = distribution_generator.rasterize_polygon(
        pit_outline, map_distance_meter, grid_size)
    grid = np.maximum(grid, pit_grid)
    grid_occupation = float(np.sum(grid) / grid.size)

    ax = distribution_generator.map_vizalization(grid, map_resolution, map_distance_meter)
    fig = plt.gcf()
    fig.savefig(raster_image_path, dpi=300)
    plt.close(fig)

    # Plan on a downsampled (obstacle-conservative) grid: pure-Python A* over
    # the full-resolution grid is prohibitively slow at this map size. Any
    # obstacle pixel in a block marks the whole coarse cell blocked, so the
    # planning grid never underestimates occupancy.
    downsample_factor = config.get("planning_downsample_factor", 1)
    planning_grid = distribution_generator.downsample_grid_max(grid, downsample_factor)
    planning_resolution = map_resolution / downsample_factor

    return {
        "grid": grid,
        "planning_grid": planning_grid,
        "planning_resolution": planning_resolution,
        "map_distance_meter": map_distance_meter,
        "map_resolution": map_resolution,
        "grid_occupation": grid_occupation,
        "num_mhp_craters": len(radius_MHP),
        # boulder positions/radii in meters, for drawing a safety-margin halo
        # around each one on top of the (per-object-identity-free) binary grid
        "boulder_centers_m": center_boulder * map_distance_meter,
        "boulder_radii_m": radius_boulder,
    }


def load_or_generate_map(config, cache_path, dist_image_path="Marius_Hills_Mapped.png",
                          raster_image_path="Marius_Hills_Rasterized.png"):
    """
    Load a previously rasterized map from cache_path (an .npz file) if it
    exists, otherwise generate it (see generate_map) and save it there for
    next time. Rasterization is the slow, deterministic-per-config part of
    the pipeline, so it's kept as one reusable artifact independent of
    whatever route is later planned across it.
    """
    cache_path = Path(cache_path)
    if cache_path.exists():
        with np.load(cache_path) as data:
            # boulder_centers_m/radii_m predate older cache files -- fall back
            # to empty arrays rather than KeyError so old caches keep loading
            # (just without the boulder safety-halo overlay until regenerated)
            has_boulders = "boulder_centers_m" in data.files
            return {
                "grid": data["grid"],
                "planning_grid": data["planning_grid"],
                "planning_resolution": float(data["planning_resolution"]),
                "map_distance_meter": float(data["map_distance_meter"]),
                "map_resolution": float(data["map_resolution"]),
                "grid_occupation": float(data["grid_occupation"]),
                "num_mhp_craters": int(data["num_mhp_craters"]),
                "boulder_centers_m": data["boulder_centers_m"] if has_boulders else np.empty((0, 2)),
                "boulder_radii_m": data["boulder_radii_m"] if has_boulders else np.empty((0,)),
            }

    map_data = generate_map(config, dist_image_path, raster_image_path)
    cache_path.parent.mkdir(parents=True, exist_ok=True)
    # compressed: the grid is a sparse binary occupancy mask (mostly zeros),
    # which compresses to a small fraction of the raw array size
    np.savez_compressed(cache_path, **map_data)
    return map_data


def run_simulation(config, dist_image_path="Marius_Hills_Mapped.png",
                    raster_image_path="Marius_Hills_Rasterized.png",
                    path_image_path="Marius_Hills_Path.png"):

    map_data = generate_map(config, dist_image_path, raster_image_path)
    planning_grid = map_data["planning_grid"]
    planning_resolution = map_data["planning_resolution"]
    map_distance_meter = map_data["map_distance_meter"]
    map_resolution = map_data["map_resolution"]
    grid_occupation = map_data["grid_occupation"]

    start_pos = (2, 2)
    goal_fraction = config["goal_fraction"]
    goal_pos = (int(map_distance_meter * planning_resolution * goal_fraction),
                int(map_distance_meter * planning_resolution * goal_fraction))

    planner = config.get("planner", "astar")
    if planner == "pyastar":
        path = path_planning_pyastar.find_path_astar(planning_grid, start_pos, goal_pos)
    elif planner == "astar":
        path = path_planning_astar.find_path_astar(planning_grid, start_pos, goal_pos)
    else:
        raise ValueError(f"Unknown planner {planner!r}, expected 'astar' or 'pyastar'")

    success = path is not None and len(path) > 0

    save_path_plot(planning_grid, path, path_image_path)

    if success:
        stats = path_analyzer.turn_analysis(path, map_resolution=planning_resolution)
    else:
        stats = {}

    result = {
        "success": success,
        "grid_occupation": grid_occupation,
        "num_mhp_craters": map_data["num_mhp_craters"],
        "planning_grid_shape": list(planning_grid.shape),
        **stats
    }

    return {
        "config": config,
        "result": result,
        # Raw path in planning-grid pixel coordinates, plus the resolution (px/m)
        # needed to interpret it, so mission planning can build drive segments
        # from this path without re-running the (slow) A* search.
        "path": path if success else None,
        "planning_resolution": planning_resolution,
    }
