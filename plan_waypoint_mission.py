from pathlib import Path
import numpy as np
from src.globalplanner import path_planning_multi
from src.operation import marius_hills_review
from src.operation import marius_hills_mission
from src.operation import path_analyzer
from src.operation import waypoint_picker
from src.operation import run_registry
from user_data.config_map import marius_hills_config
from user_data.config_mission import mission_ops_config
from user_data.config_robot import robot_power_config
from user_data.config_robot import robot_time_config
from user_data.config_robot import locomotion_experiment_config
from user_data.config_robot import robot_task_ids_config

MAP_OUTPUT_DIR = Path("experiments/marius_hills")
MAP_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
GRID_CACHE_PATH = MAP_OUTPUT_DIR / "grid_cache.npz"
LAST_WAYPOINTS_PATH = MAP_OUTPUT_DIR / "last_waypoints.json"
LAST_SPECIAL_WAYPOINTS_PATH = MAP_OUTPUT_DIR / "last_special_waypoints.json"

map_data = marius_hills_review.load_or_generate_map(
    marius_hills_config.config,
    cache_path=GRID_CACHE_PATH,
    dist_image_path=MAP_OUTPUT_DIR / "Marius_Hills_Mapped.png",
    raster_image_path=MAP_OUTPUT_DIR / "Marius_Hills_Rasterized.png",
)

# start_circle_center_m/crop_center_m default to the pit's centroid (see
# marius_hills_review._pit_centroid_m -- marked with a faint white
# circle+cross on the base map) when not explicitly set in config.
start_circle_center_m = marius_hills_config.config.get("start_circle_center_m") or map_data["pit_centroid_m"]
crop_center_m = marius_hills_config.config.get("crop_center_m") or map_data["pit_centroid_m"]

# The actual point on the start circle (same one the "circle" method below
# would compute) -- used as the landing-zone preview's center in "select"
# mode, so what you see before clicking is a real candidate landing spot,
# not the circle's abstract center.
preview_point_m = waypoint_picker.start_point_on_circle_m(
    start_circle_center_m,
    marius_hills_config.config["start_circle_radius_m"],
    marius_hills_config.config["start_angle_deg"])

# Start point is either "circle" (fixed bearing on a circle, see
# marius_hills_config.py: start_circle_center_m/radius_m/start_angle_deg)
# or "select" (click it yourself, as the first click below, before the
# waypoints -- pick_waypoints already supports this whenever start=None).
start_method = marius_hills_config.config.get("start_method", "circle")

if start_method == "select":
    start = None
else:  # "circle" (default)
    start = waypoint_picker.start_point_from_circle(
        start_circle_center_m,
        marius_hills_config.config["start_circle_radius_m"],
        marius_hills_config.config["start_angle_deg"],
        map_data["planning_resolution"])

clicked_waypoints = None
if LAST_WAYPOINTS_PATH.exists():
    saved_start, saved_waypoints = waypoint_picker.load_waypoints(LAST_WAYPOINTS_PATH)
    if saved_waypoints:
        answer = input(f"Found {len(saved_waypoints)} previously saved waypoint(s) "
                        f"({LAST_WAYPOINTS_PATH}). Use them? [Y/n]: ").strip().lower()
        if answer in ("", "y", "yes"):
            clicked_waypoints = saved_waypoints
            if start_method == "select" and saved_start is not None:
                start = saved_start  # reuse the previously clicked start point too

if clicked_waypoints is None:
    start, clicked_waypoints = waypoint_picker.pick_waypoints(
        map_data["planning_grid"], map_data["planning_resolution"], start=start,
        landing_zone_radius_m=marius_hills_config.config.get("landing_zone_radius_m"),
        start_area_radius_m=marius_hills_config.config.get("start_area_radius_m"),
        start_circle_center_m=start_circle_center_m,
        start_circle_radius_m=marius_hills_config.config.get("start_circle_radius_m"),
        preview_point_m=preview_point_m,
        boulder_centers_m=map_data.get("boulder_centers_m"),
        boulder_radii_m=map_data.get("boulder_radii_m"),
        boulder_safety_margin_m=marius_hills_config.config.get("boulder_safety_margin_m"))
    if not clicked_waypoints:
        raise ValueError("no waypoints were picked")
    waypoint_picker.save_waypoints(LAST_WAYPOINTS_PATH, start, clicked_waypoints)

if start is None:
    raise ValueError(
        "start point is None -- with start_method='select', this happens if you reused a "
        f"waypoints file saved before start-point caching existed ({LAST_WAYPOINTS_PATH}); "
        "delete it and re-run to click a start point")

special_categorized = None
if LAST_SPECIAL_WAYPOINTS_PATH.exists():
    saved_special = waypoint_picker.load_special_waypoints(LAST_SPECIAL_WAYPOINTS_PATH)
    total_marked = sum(len(wps) for wps in saved_special.values())
    if total_marked:
        answer = input(f"Found {total_marked} previously saved special waypoint(s) "
                        f"({LAST_SPECIAL_WAYPOINTS_PATH}). Use them? [Y/n]: ").strip().lower()
        if answer in ("", "y", "yes"):
            special_categorized = saved_special

if special_categorized is None:
    special_categorized = waypoint_picker.pick_special_waypoints(
        map_data["planning_grid"], map_data["planning_resolution"], clicked_waypoints, start=start,
        landing_zone_radius_m=marius_hills_config.config.get("landing_zone_radius_m"),
        start_area_radius_m=marius_hills_config.config.get("start_area_radius_m"),
        start_circle_center_m=start_circle_center_m,
        start_circle_radius_m=marius_hills_config.config.get("start_circle_radius_m"),
        boulder_centers_m=map_data.get("boulder_centers_m"),
        boulder_radii_m=map_data.get("boulder_radii_m"),
        boulder_safety_margin_m=marius_hills_config.config.get("boulder_safety_margin_m"))
    waypoint_picker.save_special_waypoints(LAST_SPECIAL_WAYPOINTS_PATH, special_categorized)

# "special" = Pit Wall Inspection triggers (phase 1/pit_ops only), see
# path_analyzer.build_drive_segments' special_trigger_idxs.
# "grav_dense_center" = anchor(s) for a denser GRAV-measurement window, see
# grav_dense_trigger_idxs/mission_ops_config.grav_dense_half_width_m.
special_waypoints = special_categorized.get("special", [])
grav_dense_waypoints = special_categorized.get("grav_dense_center", [])

waypoints = [start] + clicked_waypoints

raw_path = path_planning_multi.find_path_multi_waypoint(
    map_data["planning_grid"], waypoints, planner=marius_hills_config.config.get("planner", "pyastar"))
if raw_path is None:
    raise ValueError("no path found through the picked waypoints")

if marius_hills_config.config.get("simplify_path", True):
    # simplified per-leg (pinned on `waypoints`) so it still visits every
    # picked waypoint exactly, while every kept straight segment is
    # re-verified against the occupancy grid -- it can never cut through an
    # obstacle/border the raw A* path was avoiding
    path = path_analyzer.simplify_path_line_of_sight(raw_path, map_data["planning_grid"], pinned_points=waypoints)
    print(f"Path simplified: {len(raw_path)} -> {len(path)} points")
else:
    path = raw_path

experiment = {
    "path": path,
    "planning_resolution": map_data["planning_resolution"],
    "result": {"success": True},
    "waypoints": waypoints,
    "special_waypoints": special_waypoints,
    "grav_dense_waypoints": grav_dense_waypoints,
}

run_id, run_dir = run_registry.start_run()

# Exact vector hazard/context geometry (crater+boulder centers/radii, pit
# outline -- not the rasterized grid) for this specific run -- lightweight
# compared to the grid itself, and lets analyze_run.py re-render obstacles
# with no resolution ceiling (unlike the baked-raster background in
# Route_Plot.html/.png) without needing to reload/regenerate the whole map
# later.
np.savez_compressed(
    run_dir / "Hazards.npz",
    boulder_centers_m=map_data.get("boulder_centers_m", np.empty((0, 2))),
    boulder_radii_m=map_data.get("boulder_radii_m", np.empty((0,))),
    crater_centers_m=map_data.get("crater_centers_m", np.empty((0, 2))),
    crater_radii_m=map_data.get("crater_radii_m", np.empty((0,))),
    pit_outline_m=map_data.get("pit_outline_m", np.empty((0, 2))),
)

# Shared by every Route_Plot* variant below (only the output path and crop
# params differ per call) -- see waypoint_picker.save_route_plot/
# save_route_plot_interactive for what each of these controls.
route_plot_kwargs = dict(
    waypoints=clicked_waypoints, special_waypoints=special_categorized, start=start,
    landing_zone_radius_m=marius_hills_config.config.get("landing_zone_radius_m"),
    start_area_radius_m=marius_hills_config.config.get("start_area_radius_m"),
    start_circle_center_m=start_circle_center_m,
    start_circle_radius_m=marius_hills_config.config.get("start_circle_radius_m"),
    boulder_centers_m=map_data.get("boulder_centers_m"),
    boulder_radii_m=map_data.get("boulder_radii_m"),
    boulder_safety_margin_m=marius_hills_config.config.get("boulder_safety_margin_m"),
    corridor_width_m=marius_hills_config.config.get("robot_corridor_width_m"),
)

waypoint_picker.save_route_plot(
    map_data["planning_grid"], path, map_data["planning_resolution"], run_dir / "Route_Plot.png",
    **route_plot_kwargs)

# interactive copy alongside the PNG -- opens in a browser with the same
# scroll-to-zoom/drag-to-pan behavior as the live matplotlib window, so the
# axis stays legible at any zoom level (a static PNG/PDF can't do that: its
# ticks/detail are baked in for the full-map view)
waypoint_picker.save_route_plot_interactive(
    map_data["planning_grid"], path, map_data["planning_resolution"], run_dir / "Route_Plot.html",
    **route_plot_kwargs)

if marius_hills_config.config.get("crop_size_m"):  # crop_center_m always resolves via the pit-centroid fallback above
    waypoint_picker.save_route_plot(
        map_data["planning_grid"], path, map_data["planning_resolution"], run_dir / "Route_Plot_Crop.png",
        **route_plot_kwargs,
        crop_center_m=crop_center_m,
        crop_size_m=marius_hills_config.config["crop_size_m"],
        dpi=marius_hills_config.config.get("crop_dpi", 600))

if marius_hills_config.config.get("interactive_crop_size_m"):
    # same idea as the static crop above, but interactive: cropping to a
    # small area spends the background raster's whole pixel budget on it
    # instead of the full map, so fine detail (corridor band, small
    # boulders) is actually visible when you zoom in -- see
    # waypoint_picker.save_route_plot_interactive
    waypoint_picker.save_route_plot_interactive(
        map_data["planning_grid"], path, map_data["planning_resolution"], run_dir / "Route_Plot_Crop.html",
        **route_plot_kwargs,
        crop_center_m=crop_center_m,
        crop_size_m=marius_hills_config.config["interactive_crop_size_m"])

mission = marius_hills_mission.run_mission_planning(
    experiment, mission_ops_config.config, robot_time_config.config, robot_power_config.config,
    timeline_image_path=run_dir / "Mission_Timeline.png",
    locomotion_config=locomotion_experiment_config.config,
    task_ids_config=robot_task_ids_config.config)

print(mission["report"])

run_registry.save_run(
    base_dir="experiments/missions",
    run_id=run_id,
    run_dir=run_dir,
    config={
        "map_config": marius_hills_config.config,
        "start": start,
        "waypoints": waypoints,
        "special_waypoints": special_waypoints,
        "grav_dense_waypoints": grav_dense_waypoints,
        "mission_config": mission_ops_config.config,
        "robot_time_config": robot_time_config.config,
        "robot_power_config": robot_power_config.config,
        "locomotion_experiment_config": locomotion_experiment_config.config,
        "task_ids_config": robot_task_ids_config.config,
    },
    results={"path": path, "raw_path": raw_path, "planning_resolution": map_data["planning_resolution"], **mission},
    summary={
        "num_waypoints": len(waypoints),
        "mission_time_h": mission["report"]["mission_time_h"],
        "walking_distance_m": mission["report"]["walking_distance_m"],
    },
)

print(f"Run saved to {run_dir}")
