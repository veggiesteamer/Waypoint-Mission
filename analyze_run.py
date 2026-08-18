import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.collections import PatchCollection
from matplotlib.patches import Circle

from src.operation import conops_planner
from src.operation import path_analyzer

MISSIONS_DIR = Path("experiments/missions")

# Which run to inspect -- None picks the most recently created one (see
# view_mission_playback.py for the same convention). Set to a run_id string
# (e.g. "run_20260817_183408", the folder name under experiments/missions)
# to inspect an older run instead.
RUN_ID = None


index = json.loads((MISSIONS_DIR / "index.json").read_text())
if not index:
    raise ValueError(f"no runs found in {MISSIONS_DIR / 'index.json'}")

run_dir = Path(index[-1]["run_dir"]) if RUN_ID is None else MISSIONS_DIR / RUN_ID
print(f"Inspecting {run_dir}")

config = json.loads((run_dir / "config.json").read_text())
results = json.loads((run_dir / "results.json").read_text())
report = results["report"]

mission_config = config.get("mission_config") or {}
phase_params = mission_config.get("phase_params") or {}  # keys round-trip through JSON as strings
special_sequence_name = (phase_params.get("1") or {}).get("special_sequence_name") or "Special stops"
phase_names = tuple(mission_config.get("phase_names") or ())


# ---- console summary ----
def print_scope(name, r):
    print(f"\n{name}")
    print(f"  Mission time      : {r['mission_time_h']:.2f} h")
    print(f"  Walking distance  : {r['walking_distance_m']:.2f} m")
    print(f"  Number of stops   : {r['num_stops']}")
    if r["special_stop_count"]:
        print(f"  {special_sequence_name:18s}: {r['special_stop_count']}x")
    print(f"  Total energy      : {r['total_energy_wh']:.1f} Wh")


print_scope("Overall", report)
for phase_name, phase_report in report.get("phases", {}).items():
    print_scope(phase_name, phase_report)

waypoints = config.get("waypoints") or []
special_set = {tuple(wp) for wp in (config.get("special_waypoints") or [])}
print(f"\nRoute: {len(waypoints) - 1} waypoint(s), "
      f"{sum(1 for wp in waypoints[1:] if tuple(wp) in special_set)} marked special")


# ---- activity breakdown, as an in-memory DataFrame for further ad-hoc slicing ----
def activity_rows(scope, r):
    return [{"Scope": scope, "Activity": name, "Count": r["activity_count"][name],
              "Time (h)": r["activity_breakdown_h"][name], "Energy (Wh)": r["activity_energy_wh"][name]}
            for name in r["activity_breakdown_h"]]


phases = report.get("phases", {})
activity_df = pd.DataFrame(
    activity_rows("Overall", report) + [row for name, r in phases.items() for row in activity_rows(name, r)])
segments_df = pd.DataFrame(results["segments"])
timeline_df = pd.DataFrame(results["timeline"])

print("\nActivity breakdown (overall):")
print(activity_df[activity_df["Scope"] == "Overall"].drop(columns="Scope").to_string(index=False))

# ---- raw A* path vs. simplified line-of-sight path ----
raw_path = results.get("raw_path")
simplified_path = results["path"]
resolution = results["planning_resolution"]
window_m = mission_config.get("window_m", 1.0)
angle_threshold_deg = mission_config.get("angle_threshold_deg", 10)

if raw_path:
    path_compare_df = pd.DataFrame([
        {"Path": name, **path_analyzer.turn_analysis(p, angle_threshold_deg, resolution, window_m)}
        for name, p in (("Raw A*", raw_path), ("Simplified", simplified_path))])
    print("\nRaw vs. simplified path:")
    print(path_compare_df.to_string(index=False))

# ---- mission timeline Gantt chart -- reuses the exact plotting code the run itself used ----
tasks = [conops_planner.Task(t["name"], t["actor"], t["start"], t["end"], t["phase"]) for t in results["timeline"]]
conops_planner.plot_timeline(tasks, show=False, velocity=mission_config.get("robot_velocity"),
                              phase_names=phase_names or None,
                              time_offset_h=mission_config.get("mission_start_lunar_hour", 0.0))

# ---- activity time breakdown by phase, stacked bar ----
if phases:
    pivot_time = activity_df[activity_df["Scope"] != "Overall"].pivot_table(
        index="Activity", columns="Scope", values="Time (h)", fill_value=0)
    fig, ax = plt.subplots(figsize=(10, 6))
    pivot_time.plot(kind="bar", stacked=True, ax=ax)
    ax.set_ylabel("Time [h]")
    ax.set_title("Activity time by phase")
    plt.tight_layout()

# ---- turn density (turns per meter) along the route, special stops marked --
# normalizing by distance makes legs comparable across phases even though
# their step spacing differs (e.g. phase 0's ~6-10 m legs vs phase 1's ~1-3 m)
fig, ax1 = plt.subplots(figsize=(12, 4))
turn_density = segments_df["turns"] / segments_df["distance"]
ax1.plot(segments_df.index, turn_density, "-", color="#2a78d6", label="Turns per meter")
special_idx = segments_df.index[segments_df["special"]]
if len(special_idx):
    ax1.scatter(special_idx, turn_density.loc[special_idx], color="magenta", zorder=5,
                label=special_sequence_name)
ax1.set_xlabel("Stop index")
ax1.set_ylabel("Turns per meter")
ax1.legend(loc="upper right")
ax1.set_title("Turn density along the route")
plt.tight_layout()

# ---- raw vs. simplified path overlay, in meters -- no map background needed,
# just the two point sequences, to show how much the A* grid-staircase gets
# cleaned up without changing the actual route ----
if raw_path:
    def _path_to_m(p):
        return [(c / resolution, r / resolution) for r, c in p]

    raw_m = _path_to_m(raw_path)
    simplified_m = _path_to_m(simplified_path)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot([p[0] for p in raw_m], [p[1] for p in raw_m], "-", color="lightgray", linewidth=1,
            label=f"Raw A* ({len(raw_path)} pts)")
    ax.plot([p[0] for p in simplified_m], [p[1] for p in simplified_m], "-", color="#2a78d6", linewidth=2,
            label=f"Simplified ({len(simplified_path)} pts)")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_aspect("equal")
    ax.legend(loc="best")
    ax.set_title("Raw vs. simplified path")
    plt.tight_layout()

# ---- vectorized hazard field around the route, from the exact circle
# geometry (see plan_waypoint_mission.py's Hazards.npz) instead of a
# raster -- opened as a live matplotlib window, this has no resolution
# ceiling: zoom in as far as you want and craters/boulders stay perfectly
# sharp, unlike Route_Plot.html's baked-raster background. Restricted to
# the route's own bounding box (+ margin) since a live redraw of the full
# map's hundreds of thousands of circles would be sluggish to interact with.
hazards_path = run_dir / "Hazards.npz"
if hazards_path.exists():
    hazards = np.load(hazards_path)
    path_m = np.array([(c / resolution, r / resolution) for r, c in simplified_path])
    margin_m = 20.0
    x0, x1 = path_m[:, 0].min() - margin_m, path_m[:, 0].max() + margin_m
    y0, y1 = path_m[:, 1].min() - margin_m, path_m[:, 1].max() + margin_m

    def _in_view(centers, radii):
        if len(centers) == 0:
            return centers, radii
        mask = ((centers[:, 0] + radii >= x0) & (centers[:, 0] - radii <= x1) &
                (centers[:, 1] + radii >= y0) & (centers[:, 1] - radii <= y1))
        return centers[mask], radii[mask]

    crater_centers, crater_radii = _in_view(hazards["crater_centers_m"], hazards["crater_radii_m"])
    boulder_centers, boulder_radii = _in_view(hazards["boulder_centers_m"], hazards["boulder_radii_m"])
    # pit_outline_m predates older Hazards.npz files -- fall back gracefully
    pit_outline = hazards["pit_outline_m"] if "pit_outline_m" in hazards.files else np.empty((0, 2))

    fig, ax = plt.subplots(figsize=(9, 9))
    if len(crater_centers):
        ax.add_collection(PatchCollection(
            [Circle(c, r) for c, r in zip(crater_centers, crater_radii)],
            facecolor="tan", edgecolor="none", alpha=0.5, label=f"Craters ({len(crater_centers)})"))
    if len(boulder_centers):
        ax.add_collection(PatchCollection(
            [Circle(c, r) for c, r in zip(boulder_centers, boulder_radii)],
            facecolor="dimgray", edgecolor="none", alpha=0.7, label=f"Boulders ({len(boulder_centers)})"))
    if len(pit_outline):
        ax.plot(np.append(pit_outline[:, 0], pit_outline[0, 0]),
                np.append(pit_outline[:, 1], pit_outline[0, 1]),
                "-", color="firebrick", linewidth=2, label="Pit outline")
    if raw_path:
        raw_path_m = np.array([(c / resolution, r / resolution) for r, c in raw_path])
        ax.plot(raw_path_m[:, 0], raw_path_m[:, 1], "-", color="lightgray", linewidth=1,
                label=f"Raw A* ({len(raw_path)} pts)")
    ax.plot(path_m[:, 0], path_m[:, 1], "-", color="#2a78d6", linewidth=2, label="Path")
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.legend(loc="best")
    ax.set_title("Hazards along the route (vectorized -- zoom freely, no resolution limit)")
    plt.tight_layout()
else:
    print(f"\nNo {hazards_path.name} for this run (predates hazard export) -- skipping vectorized hazard plot")

plt.show()
