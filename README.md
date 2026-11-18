# Traversability Assessment

## South Pole baseline mission (current work)

`south_pole_sim.py` checks whether the science team's South Pole baseline mission
fits one lunar day, and how much time, energy and data it needs. There is no map or
path planning: walking time is distance × tortuosity ÷ speed.

```
python south_pole_sim.py [user_data/Input_Conops_v12.xlsx]
```

Everything is defined in the input workbook (`user_data/Input_Conops_v12.xlsx`):
science table (Mission) → Activity map → Procedures → Modes, plus Phases and
Parameters. The workbook's README and Open items sheets explain the model and list
what is still missing. Formulas refer to parameters by name (e.g. `=90/turn_rate`)
and are evaluated by the script, so the file does not need to be re-saved in Excel.

Each run writes `experiments/south_pole/run_<timestamp>/` (not tracked by git):

| File | Content |
|---|---|
| `summary.txt` | feasibility vs. the lunar-day window, energy, data, per-phase / per-mode / per-activity tables, images per science ID, outage-timing sweep, workbook consistency checks |
| `stats.png` | one-page key numbers: stat tiles, time split, energy per mode |
| `timeline_phases.png` | one panel per phase on a common time scale |
| `data_overview.png` | onboard data (science / operator / housekeeping) over the mission |
| `timeline.csv` | every step with start, duration, mode, power, energy, distance, data |

Model in short: the robot runs strictly sequentially; every station walks, settles,
takes NAV + full-resolution SciCAM + a compressed operator image, sends the operator
data, does its assigned observations, naps, waits for the ground only if the decision
is not back yet, then offloads. The ground decision runs in parallel. A 4 h comms
outage every 24 h blocks downlink, ground decisions and walking.

The Marius Hills waypoint workflow below is the previous mission and is kept for reference.

---

Lunar rover traversability and global path planning: hazard maps (craters, boulders,
pit outlines) are rasterized into an occupancy grid, a rover route is planned across
it with A*, and a mission timeline (drive/turn/downlink/measurement/nap) is derived
from the resulting path.

This document focuses on the **Marius Hills Pit waypoint mission workflow**, the
main entry point for planning a rover route by hand and getting a mission timeline
out of it. Other root scripts (scenario/sensitivity sweeps, the demo) are covered
briefly at the end.

## Setup

```
pip install -r requirements.txt
```

`pyastar2d` (the fast compiled A* backend) is optional but recommended — without it,
set `"planner": "astar"` in the map config to fall back to the slower pure-Python
planner.

## Waypoint mission workflow

```
python plan_waypoint_mission.py
```

One script, run start to finish, that:

1. **Gets the rasterized map.** Checks `experiments/marius_hills/grid_cache.npz`.
   - Exists → loads it instantly.
   - Doesn't exist → rasterizes all hazard layers from scratch (slow — this is a
     pure-Python loop over every crater/boulder/pit cell) and saves the result there
     (compressed — the grid is mostly zeros, so this is a small fraction of the raw
     array size).
   - The cache does **not** know if you changed `marius_hills_config.py`. If you
     edit map size, resolution, or hazard parameters, delete `grid_cache.npz`
     yourself to force a fresh rasterization.

2. **Computes the start point.** Not clicked — fixed at a bearing on a circle
   around a center point: `start_circle_center_m`, `start_circle_radius_m`, and
   `start_angle_deg` in `marius_hills_config.py` (angle measured counterclockwise
   from +X/east, standard math convention). Change `start_angle_deg` and re-run to
   move the start point around the circle. The circle (dashed cyan) and the landing
   zone / start area circles around the resulting point are drawn on the map for
   reference.

3. **Gets your mission waypoints.** Checks `experiments/marius_hills/last_waypoints.json`.
   - Exists → prompts in the terminal:
     `Found N previously saved waypoint(s)... Use them? [Y/n]`.
     Enter or `y` reuses them; anything else opens the picker.
   - Doesn't exist → opens the picker directly.
   - A matplotlib window shows the rasterized map (axes in meters) with the fixed
     start point already marked, plus a faint gray safety-margin halo around each
     boulder (`boulder_safety_margin_m` in the config — visual context only, not
     enforced during planning). Click to place mission waypoints in order, with a
     line growing from the start to show the route being built. Use the toolbar to
     zoom/pan. Press `z` to undo the last waypoint. Close the window when done.
     Whatever you pick overwrites `last_waypoints.json` (alongside the start point,
     for the record — the start itself is always recomputed from config, not
     reused from this file).

4. **Plans the route.** Chains A* legs: start point → waypoint 1 → waypoint 2 →
   ... in the order you picked/loaded, using `find_path_multi_waypoint`.

4b. **Simplifies the path** (`simplify_path: True` in `marius_hills_config.py`,
   on by default). A* only moves in 8 directions per grid cell, so the raw path
   is a dense staircase of tiny steps. `path_analyzer.simplify_path_line_of_sight`
   greedily collapses it into the fewest straight segments such that each
   segment is individually re-checked against the occupancy grid — so it can
   never cut through an obstacle/border the raw path was avoiding — while every
   picked waypoint is pinned and always remains an exact vertex of the result.
   Prints `Path simplified: N -> M points` to the terminal.

5. **Starts a new run folder**: `experiments/missions/run_<timestamp>/` — every
   execution gets its own folder, whether waypoints were reused or freshly picked.

6. **Saves a picture of the planned route** → `Route_Plot.png`, plus a
   high-resolution crop → `Route_Plot_Crop.png` (a `crop_size_m`-wide square
   around `crop_center_m`, rendered at `crop_dpi`; the crop is skipped if
   either config value is missing). Both also show a faint gray corridor of
   width `robot_corridor_width_m` along the whole route — the rover's own
   footprint/safety margin as it drives — alongside the boulder halos.

7. **Runs the mission-timeline logic** (turns, drive time, stops, power) →
   `Mission_Timeline.png`. Each "drive" bar is now annotated with the distance
   covered in that leg, and the plot title shows the total distance driven.
   Prints a summary report to the terminal.

8. **Saves provenance**: `config.json` (map config + start + waypoints +
   mission/robot-time/robot-power config — everything needed to exactly
   reproduce this run) and `results.json` (path + segments + report + the
   full serialized timeline), and appends one entry to
   `experiments/missions/index.json` so any past run can be traced back to
   the parameters that produced it.

### Mission playback (experimental)

```
python view_mission_playback.py
```

Loads the most recent run from `experiments/missions/index.json` and opens a
two-panel window: the map + planned route on top, the task timeline on the
bottom. **Move the mouse over the timeline** to see a magenta marker jump to
the rover's position on the map at that moment, with elapsed time and distance
driven shown in the corner — a way to scrub through the mission and see where
the rover physically is at any point in the schedule. This is a new,
lightly-tested feature (`src/operation/mission_playback.py`); it reconstructs
the rover's position by interpolating along the planned path using cumulative
drive distance, so it's exact for the "drive" phases and holds position during
stationary tasks (picture, downlink, relay, measurement, nap).

### Output layout

```
experiments/
├── marius_hills/
│   ├── grid_cache.npz          # rasterized map, reused every run
│   └── last_waypoints.json     # most recent waypoint pick, reused/replaced every run
└── missions/
    ├── index.json               # one row per run: timestamp, folder, summary
    └── run_20260811_143000/
        ├── config.json
        ├── results.json
        ├── Route_Plot.png
        ├── Route_Plot_Crop.png
        └── Mission_Timeline.png
```

### Config files used

| File | Purpose |
|---|---|
| `user_data/config_map/marius_hills_config.py` | Map size/resolution, hazard CSV paths (`csv_path`, `pit_outline_path`), synthetic crater/boulder distribution params, landing zone/start area radii, start-circle position (`start_circle_center_m`, `start_circle_radius_m`, `start_angle_deg`), high-res crop settings (`crop_center_m`, `crop_size_m`, `crop_dpi`), boulder/corridor safety-margin visuals (`boulder_safety_margin_m`, `robot_corridor_width_m`), planner choice, `simplify_path` toggle. |
| `user_data/config_mission/mission_ops_config.py` | Rover velocity, stop-distance range, turn-detection thresholds. |
| `user_data/config_robot/robot_time_config.py` | Per-activity durations (turn, picture, downlink, relay, measurement, nap, uplink). |
| `user_data/config_robot/robot_power_config.py` | Power model parameters (currently accepted but not yet used in the timeline calc). |

### Key modules

| Module | Role |
|---|---|
| `src/operation/marius_hills_review.py` | `generate_map()`/`load_or_generate_map()` — hazard loading (real craters, pit outline, synthetic craters/boulders) and rasterization. |
| `src/globalplanner/path_planning_multi.py` | `find_path_multi_waypoint()` — chains single-pair A* (`path_planning_astar.py` / `path_planning_pyastar.py`) across an ordered waypoint list. |
| `src/operation/waypoint_picker.py` | Interactive click-to-place picker, route-plot saving, waypoint save/load. |
| `src/operation/run_registry.py` | `start_run()`/`save_run()` — per-run provenance folder + index. |
| `src/operation/path_analyzer.py` | Turn detection, drive-leg segmentation (`build_drive_segments`), and grid-aware line-of-sight path simplification (`simplify_path_line_of_sight`) from a raw path. |
| `src/operation/conops_planner.py` | Builds the task timeline (turn/drive/picture/downlink/relay/measurement/nap) from drive segments. |
| `src/operation/marius_hills_mission.py` | `run_mission_planning()` — ties `path_analyzer` + `conops_planner` together and prints/returns the mission summary. |
| `src/operation/mission_playback.py` | Experimental. Time↔distance↔position mapping and the interactive timeline/map hover viewer. |

## Other root scripts

These are separate, older pipelines, unrelated to the waypoint workflow above:

- **`path_planning_marius_hills.py`** + **`mission_planning_marius_hills.py`** — the
  original two-script version: plans a single fixed lander→pit path
  (`goal_fraction` in the map config), saves it to
  `experiments/marius_hills/marius_hills_experiment.json`, then a second script
  builds a mission timeline from that saved path. Use this only if you want that
  one fixed destination instead of a custom waypoint route.
- **`run_scenario_comparison.py`** — runs several named scenarios (see
  `user_data/config_map/scenarios_config.py`) through the synthetic (non-Marius-Hills)
  pipeline and saves all results to `experiments/scenario_comparison/`.
- **`run_sensitivity_sweep.py`** / **`plot_sensitivity_sweep_existing.py`** —
  parameter-sweep runs and plotting of previously swept results.
- **`path_planning_demo.py`** — minimal single-path demo on synthetic hazards, no
  saved config/results, just map/path images.
