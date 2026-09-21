import json
from pathlib import Path

from src.operation import marius_hills_review
from src.operation import mission_playback

MAP_OUTPUT_DIR = Path("experiments/marius_hills")
GRID_CACHE_PATH = MAP_OUTPUT_DIR / "grid_cache.npz"
MISSIONS_DIR = Path("experiments/missions")

index = json.loads((MISSIONS_DIR / "index.json").read_text())
if not index:
    raise ValueError(f"no runs found in {MISSIONS_DIR / 'index.json'}")

run_dir = Path(index[-1]["run_dir"])  # most recent run
print(f"Loading run: {run_dir}")

config = json.loads((run_dir / "config.json").read_text())
results = json.loads((run_dir / "results.json").read_text())

map_config = config["map_config"]
map_data = marius_hills_review.load_or_generate_map(map_config, cache_path=GRID_CACHE_PATH)

start = tuple(config["start"])
waypoints = [tuple(wp) for wp in config["waypoints"][1:]]  # exclude start, it's passed separately
path = [tuple(p) for p in results["path"]]
resolution = results.get("planning_resolution", map_data["planning_resolution"])
velocity = config["mission_config"]["robot_velocity"]

mission_playback.show_mission_playback(
    map_data["planning_grid"], path, resolution, results["timeline"], velocity,
    waypoints=waypoints, start=start,
    landing_zone_radius_m=map_config.get("landing_zone_radius_m"),
    start_area_radius_m=map_config.get("start_area_radius_m"),
    start_circle_center_m=map_config.get("start_circle_center_m"),
    start_circle_radius_m=map_config.get("start_circle_radius_m"),
    boulder_centers_m=map_data.get("boulder_centers_m"),
    boulder_radii_m=map_data.get("boulder_radii_m"),
    boulder_safety_margin_m=map_config.get("boulder_safety_margin_m"),
    corridor_width_m=map_config.get("robot_corridor_width_m"),
    crop_center_m=map_config.get("crop_center_m"),
    crop_size_m=map_config.get("crop_size_m"),
    time_offset_h=config["mission_config"].get("mission_start_lunar_hour", 0.0))
