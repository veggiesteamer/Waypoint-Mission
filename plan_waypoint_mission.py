from pathlib import Path

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

MAP_OUTPUT_DIR = Path("experiments/marius_hills")
MAP_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
GRID_CACHE_PATH = MAP_OUTPUT_DIR / "grid_cache.npz"
LAST_WAYPOINTS_PATH = MAP_OUTPUT_DIR / "last_waypoints.json"

map_data = marius_hills_review.load_or_generate_map(
    marius_hills_config.config,
    cache_path=GRID_CACHE_PATH,
    dist_image_path=MAP_OUTPUT_DIR / "Marius_Hills_Mapped.png",
    raster_image_path=MAP_OUTPUT_DIR / "Marius_Hills_Rasterized.png",
)

# Start point is fixed at a given bearing on a circle around a center point
# (see marius_hills_config.py: start_circle_center_m/radius_m/start_angle_deg)
# rather than clicked freely.
start = waypoint_picker.start_point_from_circle(
    marius_hills_config.config["start_circle_center_m"],
    marius_hills_config.config["start_circle_radius_m"],
    marius_hills_config.config["start_angle_deg"],
    map_data["planning_resolution"])

clicked_waypoints = None
if LAST_WAYPOINTS_PATH.exists():
    _, saved_waypoints = waypoint_picker.load_waypoints(LAST_WAYPOINTS_PATH)
    if saved_waypoints:
        answer = input(f"Found {len(saved_waypoints)} previously saved waypoint(s) "
                        f"({LAST_WAYPOINTS_PATH}). Use them? [Y/n]: ").strip().lower()
        if answer in ("", "y", "yes"):
            clicked_waypoints = saved_waypoints

if clicked_waypoints is None:
    _, clicked_waypoints = waypoint_picker.pick_waypoints(
        map_data["planning_grid"], map_data["planning_resolution"], start=start,
        landing_zone_radius_m=marius_hills_config.config.get("landing_zone_radius_m"),
        start_area_radius_m=marius_hills_config.config.get("start_area_radius_m"),
        start_circle_center_m=marius_hills_config.config.get("start_circle_center_m"),
        start_circle_radius_m=marius_hills_config.config.get("start_circle_radius_m"),
        boulder_centers_m=map_data.get("boulder_centers_m"),
        boulder_radii_m=map_data.get("boulder_radii_m"),
        boulder_safety_margin_m=marius_hills_config.config.get("boulder_safety_margin_m"))
    if not clicked_waypoints:
        raise ValueError("no waypoints were picked")
    waypoint_picker.save_waypoints(LAST_WAYPOINTS_PATH, start, clicked_waypoints)

waypoints = [start] + clicked_waypoints

path = path_planning_multi.find_path_multi_waypoint(
    map_data["planning_grid"], waypoints, planner=marius_hills_config.config.get("planner", "pyastar"))
if path is None:
    raise ValueError("no path found through the picked waypoints")

if marius_hills_config.config.get("simplify_path", True):
    raw_path_length = len(path)
    # simplified per-leg (pinned on `waypoints`) so it still visits every
    # picked waypoint exactly, while every kept straight segment is
    # re-verified against the occupancy grid -- it can never cut through an
    # obstacle/border the raw A* path was avoiding
    path = path_analyzer.simplify_path_line_of_sight(path, map_data["planning_grid"], pinned_points=waypoints)
    print(f"Path simplified: {raw_path_length} -> {len(path)} points")

experiment = {
    "path": path,
    "planning_resolution": map_data["planning_resolution"],
    "result": {"success": True},
}

run_id, run_dir = run_registry.start_run()

waypoint_picker.save_route_plot(
    map_data["planning_grid"], path, map_data["planning_resolution"], run_dir / "Route_Plot.png",
    waypoints=clicked_waypoints, start=start,
    landing_zone_radius_m=marius_hills_config.config.get("landing_zone_radius_m"),
    start_area_radius_m=marius_hills_config.config.get("start_area_radius_m"),
    start_circle_center_m=marius_hills_config.config.get("start_circle_center_m"),
    start_circle_radius_m=marius_hills_config.config.get("start_circle_radius_m"),
    boulder_centers_m=map_data.get("boulder_centers_m"),
    boulder_radii_m=map_data.get("boulder_radii_m"),
    boulder_safety_margin_m=marius_hills_config.config.get("boulder_safety_margin_m"),
    corridor_width_m=marius_hills_config.config.get("robot_corridor_width_m"))

if marius_hills_config.config.get("crop_center_m") and marius_hills_config.config.get("crop_size_m"):
    waypoint_picker.save_route_plot(
        map_data["planning_grid"], path, map_data["planning_resolution"], run_dir / "Route_Plot_Crop.png",
        waypoints=clicked_waypoints, start=start,
        landing_zone_radius_m=marius_hills_config.config.get("landing_zone_radius_m"),
        start_area_radius_m=marius_hills_config.config.get("start_area_radius_m"),
        start_circle_center_m=marius_hills_config.config.get("start_circle_center_m"),
        start_circle_radius_m=marius_hills_config.config.get("start_circle_radius_m"),
        boulder_centers_m=map_data.get("boulder_centers_m"),
        boulder_radii_m=map_data.get("boulder_radii_m"),
        boulder_safety_margin_m=marius_hills_config.config.get("boulder_safety_margin_m"),
        corridor_width_m=marius_hills_config.config.get("robot_corridor_width_m"),
        crop_center_m=marius_hills_config.config["crop_center_m"],
        crop_size_m=marius_hills_config.config["crop_size_m"],
        dpi=marius_hills_config.config.get("crop_dpi", 600))

mission = marius_hills_mission.run_mission_planning(
    experiment, mission_ops_config.config, robot_time_config.config, robot_power_config.config,
    timeline_image_path=run_dir / "Mission_Timeline.png")

print(mission["report"])

run_registry.save_run(
    base_dir="experiments/missions",
    run_id=run_id,
    run_dir=run_dir,
    config={
        "map_config": marius_hills_config.config,
        "start": start,
        "waypoints": waypoints,
        "mission_config": mission_ops_config.config,
        "robot_time_config": robot_time_config.config,
        "robot_power_config": robot_power_config.config,
    },
    results={"path": path, "planning_resolution": map_data["planning_resolution"], **mission},
    summary={
        "num_waypoints": len(waypoints),
        "mission_time_h": mission["report"]["mission_time_h"],
        "walking_distance_m": mission["report"]["walking_distance_m"],
    },
)

print(f"Run saved to {run_dir}")
