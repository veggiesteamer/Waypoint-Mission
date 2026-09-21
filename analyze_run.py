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
grav_dense_set = {tuple(wp) for wp in (config.get("grav_dense_waypoints") or [])}
print(f"\nRoute: {len(waypoints) - 1} waypoint(s), "
      f"{sum(1 for wp in waypoints[1:] if tuple(wp) in special_set)} marked special, "
      f"{sum(1 for wp in waypoints[1:] if tuple(wp) in grav_dense_set)} marked GRAV Dense Center")


# ---- activity breakdown, as an in-memory DataFrame for further ad-hoc slicing ----
# one Task ID per display name (e.g. "GRAV measurement" -> "OPS-13") -- see
# user_data/config_robot/robot_task_ids_config.py. .get("task_id") guards
# older runs whose saved timeline predates this field.
name_to_id = {t["name"]: t.get("task_id") for t in results["timeline"]}


def activity_rows(scope, r):
    distance_m = r["walking_distance_m"]
    return [{"Scope": scope, "Activity": name, "ID": name_to_id.get(name),
              "Count": r["activity_count"][name],
              "Per meter": r["activity_count"][name] / distance_m if distance_m else 0.0,
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

# ---- high-level mission timeline: named phase blocks (Egress,
# Commissioning, Traverse, each Pit Survey/Tech Demo pair, the Dense Grav
# window, and the pit_ops driving stretches around them), one row per mode
# in mission-clock time shifted to real lunar-day time -- a coarse
# companion to the full Gantt chart above ----
def _tasks_named(name):
    return sorted((t for t in results["timeline"] if t["name"] == name), key=lambda t: t["start"])


walk_tasks = _tasks_named("Walk")
loco_tasks = _tasks_named("Locomotion Test")
mission_end_s = max(t["end"] for t in results["timeline"])

# Each segment's own span: its Walk task's start to the next segment's Walk
# task start (mission end for the last) -- naturally covers that stop's
# full task set (drive, turn, standing, measurements, nap, downlink). A
# special stop's span gets capped at its own following Locomotion Test
# ("Tech Demo" here) task's start (if any), so that block is carved out as
# its own entry below instead of being silently absorbed into the survey's
# own time.
records = []
survey_count = 0
loco_ptr = 0
for i, seg in enumerate(results["segments"]):
    span_start = walk_tasks[i]["start"]
    span_end = walk_tasks[i + 1]["start"] if i + 1 < len(walk_tasks) else mission_end_s
    # subphase is checked first, regardless of phase -- the Dense Grav
    # window can fall inside either the traverse or the pit_ops phase
    # depending on where the marked waypoint sits along the route, and
    # should show up as its own row either way rather than being absorbed
    # into whichever phase it happens to land in
    if seg.get("subphase") == "grav_dense_center":
        label, end = "Dense Grav", span_end
    elif seg.get("phase", 0) == 0:
        label, end = "Traverse", span_end
    elif seg.get("special"):
        survey_count += 1
        label, end = f"Pit Survey {survey_count}", span_end
        if loco_ptr < len(loco_tasks) and span_start <= loco_tasks[loco_ptr]["start"] < span_end:
            end = loco_tasks[loco_ptr]["start"]
    else:
        label, end = "Pit ops", span_end
    records.append((label, span_start, end))
    if seg.get("special") and end < span_end:
        lt = loco_tasks[loco_ptr]
        records.append((f"Tech Demo {survey_count}", lt["start"], lt["end"]))
        loco_ptr += 1

# merge consecutive same-label records (Traverse/Pit ops/Dense Grav can span
# several segments; Pit Survey/Tech Demo labels are already unique per
# occurrence, so they never merge with each other)
high_level_blocks = []
for label, start, end in records:
    if high_level_blocks and high_level_blocks[-1][0] == label:
        high_level_blocks[-1] = (label, high_level_blocks[-1][1], end)
    else:
        high_level_blocks.append((label, start, end))

egress_tasks, commissioning_tasks = _tasks_named("Egress"), _tasks_named("Commissioning")
if commissioning_tasks:
    high_level_blocks.insert(0, ("Commissioning", commissioning_tasks[0]["start"], commissioning_tasks[0]["end"]))
if egress_tasks:
    high_level_blocks.insert(0, ("Egress", egress_tasks[0]["start"], egress_tasks[0]["end"]))


def _mode(label):
    # collapse an occurrence-numbered label ("Pit Survey 2") down to its
    # row/mode name ("Pit Survey") -- Egress/Commissioning/Traverse/Pit
    # ops/Dense Grav are never numbered and pass through unchanged
    prefix, _, suffix = label.rpartition(" ")
    return prefix if suffix.isdigit() else label


def _mode_color(mode):
    return {
        "Egress": "#898781",
        "Commissioning": "#c3c2b7",
        "Traverse": "brown",
        "Pit Survey": "#eb6834",
        "Tech Demo": "blue",
        "Pit ops": "yellow",
        "Dense Grav": "green",
    }.get(mode, "#52514e")


total_h = mission_end_s / 3600
mission_start_lunar_hour = mission_config.get("mission_start_lunar_hour", 0.0)


def _to_lunar_h(t_seconds):
    return t_seconds / 3600 + mission_start_lunar_hour


print("\nHigh-level timeline:")
for label, start, end in high_level_blocks:
    print(f"  {label:18s}: {_to_lunar_h(start):6.2f} - {_to_lunar_h(end):6.2f} h (lunar)  "
          f"({(end - start) / 3600:5.2f} h, {100 * (end - start) / 3600 / total_h:4.1f}%)")

# one row per mode, in the order each first appears (matches
# conops_planner.plot_timeline's own row-per-task-name convention -- bars
# for the same mode land on the same row regardless of how many times it
# recurs, e.g. both Pit Surveys and both Tech Demos)
mode_order = list(dict.fromkeys(_mode(label) for label, _, _ in high_level_blocks))
y_map = {mode: i for i, mode in enumerate(mode_order)}

# Sizing/font knobs for this plot -- tune to taste. FIG_WIDTH_IN is the
# figure's width; FIG_ROW_HEIGHT_IN is how much height each mode row adds
# (total figure height = FIG_ROW_HEIGHT_IN * number of rows + FIG_HEIGHT_
# BASE_IN); the *_FONTSIZE knobs cover the title, the axis label, the tick
# labels (mode names on the y-axis, hour numbers on the x-axis), and the
# per-bar duration label (BAR_LABEL_FONTSIZE).
FIG_WIDTH_IN = 14
FIG_ROW_HEIGHT_IN = 0.6
FIG_HEIGHT_BASE_IN = 1
TITLE_FONTSIZE = 20
AXIS_LABEL_FONTSIZE = 18
TICK_FONTSIZE = 18
BAR_LABEL_FONTSIZE = 11

fig, ax = plt.subplots(figsize=(FIG_WIDTH_IN, FIG_ROW_HEIGHT_IN * len(mode_order) + FIG_HEIGHT_BASE_IN))
for label, start, end in high_level_blocks:
    mode = _mode(label)
    start_h, dur_h = _to_lunar_h(start), (end - start) / 3600
    ax.broken_barh([(start_h, dur_h)], (y_map[mode] - 0.4, 0.8),
                    facecolor=_mode_color(mode), edgecolor=_mode_color(mode))
    # duration label, centered above each bar (not inside it -- a lot of
    # these bars, e.g. Pit Survey/Tech Demo/Dense Grav, are only a couple
    # of hours wide against a total mission of ~150h+, far too narrow for
    # inline text) -- sits in the gap between this row and the next, which
    # every row reserves (bars are 0.8 units tall, rows are 1 unit apart).
    # That gap is only 0.2 units, tight enough that a label can still
    # brush the row above it when several narrow bars land close together
    # in time (e.g. a Pit Survey right next to its Tech Demo) -- the white
    # backing box keeps it legible either way instead of relying on exact
    # spacing math.
    ax.text(start_h + dur_h / 2, y_map[mode] - 0.45, f"{dur_h:.1f} h",
             ha="center", va="bottom", fontsize=BAR_LABEL_FONTSIZE,
             bbox=dict(boxstyle="round,pad=0.15", facecolor="white", edgecolor="none", alpha=0.8))

ax.set_yticks(list(y_map.values()))
ax.set_yticklabels(list(y_map.keys()), fontsize=TICK_FONTSIZE)
ax.invert_yaxis()
ax.axvline(mission_start_lunar_hour, color="black", linewidth=1)
ax.set_xlim(_to_lunar_h(0) - total_h * 0.02, _to_lunar_h(mission_end_s) + total_h * 0.02)
ax.tick_params(axis="x", labelsize=TICK_FONTSIZE)
ax.set_xlabel("Mission time [h]", fontsize=AXIS_LABEL_FONTSIZE)
ax.set_title("High-level mission timeline", fontsize=TITLE_FONTSIZE)
plt.tight_layout()

# ---- mission time vs. available lunar-daylight budget -- two bars, both
# starting at lunar hour 0 (daylight start): the top one is the same
# landing-to-pit_ops story as the plot above, collapsed to 5 blocks (no
# Dense Grav/Pit Survey/Tech Demo split -- those all fold into whichever
# phase they're physically part of here); the bottom one is the full
# daylight window available before lunar night, with the unused remainder
# (mission end -> end of daylight) marked as a hatched "Margin" segment so
# it reads as reserve, not part of the plan ----
phase0_count = sum(1 for seg in results["segments"] if seg.get("phase", 0) == 0)
traverse_end_s = walk_tasks[phase0_count]["start"] if phase0_count < len(walk_tasks) else mission_end_s

landing_dur_h = mission_start_lunar_hour
egress_dur_h = (egress_tasks[0]["end"] - egress_tasks[0]["start"]) / 3600 if egress_tasks else 0.0
commissioning_dur_h = (
    (commissioning_tasks[0]["end"] - commissioning_tasks[0]["start"]) / 3600 if commissioning_tasks else 0.0)
traverse_dur_h = (traverse_end_s - walk_tasks[0]["start"]) / 3600
pitops_dur_h = (mission_end_s - traverse_end_s) / 3600

mission_blocks_simple = [
    ("Landing", landing_dur_h),
    ("Egress", egress_dur_h),
    ("Commissioning", commissioning_dur_h),
    ("Traverse", traverse_dur_h),
    ("Pit ops", pitops_dur_h),
]
mission_total_h = sum(dur for _, dur in mission_blocks_simple)

# Daylight half of the lunar day -- the actual operational budget (a rover
# is solar-powered, so lunar night is unavailable regardless of how long
# the full day/night cycle is).
available_h = mission_config.get("lunar_day_hours", 708.7) / 2

BUDGET_FIG_WIDTH_IN = FIG_WIDTH_IN
BUDGET_FIG_HEIGHT_IN = 4.5
BUDGET_TITLE_FONTSIZE = TITLE_FONTSIZE
BUDGET_AXIS_LABEL_FONTSIZE = AXIS_LABEL_FONTSIZE
BUDGET_TICK_FONTSIZE = TICK_FONTSIZE
BUDGET_BAR_LABEL_FONTSIZE = BAR_LABEL_FONTSIZE

fig, ax = plt.subplots(figsize=(BUDGET_FIG_WIDTH_IN, BUDGET_FIG_HEIGHT_IN))
y_mission, y_available = 0, 1

# Blocks on the Mission bar sit flush against each other (a literal
# stacked/sequential bar, unlike the per-mode Gantt above where different
# modes had their own row) -- Landing/Egress/Commissioning in particular
# are short enough that their labels collide even though the blocks
# themselves don't. Stagger labels upward (with a leader line back to
# their block) whenever a new label's x lands close to an already-placed
# one, same collision-aware approach used for the categorical timeline
# further up before it moved to one-row-per-mode.
label_level_step = 0.35
recent_label_x = []
max_label_level = 0


def _place_label(mid_x, y_bar_top, text):
    global max_label_level
    threshold = available_h * 0.06
    level = sum(1 for x in recent_label_x if abs(mid_x - x) < threshold)
    recent_label_x.append(mid_x)
    max_label_level = max(max_label_level, level)
    y_text = y_bar_top - 0.1 - level * label_level_step
    ax.annotate(text, xy=(mid_x, y_bar_top), xytext=(mid_x, y_text), ha="center", va="bottom",
                fontsize=BUDGET_BAR_LABEL_FONTSIZE,
                bbox=dict(boxstyle="round,pad=0.15", facecolor="white", edgecolor="none", alpha=0.8),
                arrowprops=dict(arrowstyle="-", color="#898781", lw=0.8))


t = 0.0
for label, dur in mission_blocks_simple:
    if dur <= 0:
        continue
    color = "#5b5b5b" if label == "Landing" else _mode_color(label)
    ax.broken_barh([(t, dur)], (y_mission - 0.4, 0.8), facecolor=color, edgecolor=color)
    _place_label(t + dur / 2, y_mission - 0.4, f"{label}: {dur:.1f} h")
    t += dur

ax.broken_barh([(0, mission_total_h)], (y_available - 0.4, 0.8), facecolor="#b0b0b0", edgecolor="#b0b0b0")
_place_label(mission_total_h / 2, y_available - 0.4, f"Used: {mission_total_h:.1f} h")
margin_h = available_h - mission_total_h
if margin_h > 0:
    ax.broken_barh([(mission_total_h, margin_h)], (y_available - 0.4, 0.8),
                    facecolor="white", edgecolor="gray", hatch="//")
    _place_label(mission_total_h + margin_h / 2, y_available - 0.4, f"Margin: {margin_h:.1f} h")

ax.set_yticks([y_mission, y_available])
ax.set_yticklabels(["Mission", "Available (daylight)"], fontsize=BUDGET_TICK_FONTSIZE)
# explicit inverted limits (not just invert_yaxis()) so there's headroom
# above the Mission row's labels -- otherwise they crowd the title,
# especially at larger BUDGET_BAR_LABEL_FONTSIZE/BUDGET_TITLE_FONTSIZE.
# Scales with however many stagger levels the label collision avoidance
# above actually needed.
ax.set_ylim(y_available + 0.7, y_mission - 0.5 - (max_label_level + 1) * label_level_step)
ax.set_xlim(-available_h * 0.02, available_h * 1.05)
ax.tick_params(axis="x", labelsize=BUDGET_TICK_FONTSIZE)
ax.set_xlabel("Time since daylight start [h]", fontsize=BUDGET_AXIS_LABEL_FONTSIZE)
ax.set_title("Mission time vs. available lunar-daylight budget", fontsize=BUDGET_TITLE_FONTSIZE)
plt.tight_layout()

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
if "subphase" in segments_df.columns:  # predates older saved runs
    subphase_idx = segments_df.index[segments_df["subphase"].notna()]
    if len(subphase_idx):
        ax1.scatter(subphase_idx, turn_density.loc[subphase_idx], color="cyan", zorder=5,
                    label="Subphase window")
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

    # Sizing/font knobs for this plot -- tune to taste, same idea as the
    # high-level timeline plot above (HAZARD_-prefixed here since this is a
    # separate figure with its own settings).
    HAZARD_FIG_SIZE_IN = 9
    HAZARD_TITLE_FONTSIZE = TITLE_FONTSIZE
    HAZARD_AXIS_LABEL_FONTSIZE = AXIS_LABEL_FONTSIZE
    HAZARD_TICK_FONTSIZE = TICK_FONTSIZE
    HAZARD_LEGEND_FONTSIZE = BAR_LABEL_FONTSIZE 



    fig, ax = plt.subplots(figsize=(HAZARD_FIG_SIZE_IN, HAZARD_FIG_SIZE_IN))
    if len(crater_centers):
        ax.add_collection(PatchCollection(
            [Circle(c, r) for c, r in zip(crater_centers, crater_radii)],
            facecolor="tan", edgecolor="none", alpha=0.5, label=f"Craters ({len(crater_centers)})"))
    if len(boulder_centers):
        ax.add_collection(PatchCollection(
            [Circle(c, r) for c, r in zip(boulder_centers, boulder_radii)],
            facecolor="dimgray", edgecolor="none", alpha=0.7, label=f"Boulders ({len(boulder_centers)})"))

    #if len(pit_outline):
    #    ax.plot(np.append(pit_outline[:, 0], pit_outline[0, 0]),
    #            np.append(pit_outline[:, 1], pit_outline[0, 1]),
    #            "-", color="firebrick", linewidth=2, label="Pit outline")

    if raw_path:
        raw_path_m = np.array([(c / resolution, r / resolution) for r, c in raw_path])
        ax.plot(raw_path_m[:, 0], raw_path_m[:, 1], "-", color="firebrick", linewidth=1,
                label=f"Raw A* ({len(raw_path)} pts)")
    ax.plot(path_m[:, 0], path_m[:, 1], "-", color="#2a78d6", linewidth=1, label="Path")
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.set_xlabel("X [m]", fontsize=HAZARD_AXIS_LABEL_FONTSIZE)
    ax.set_ylabel("Y [m]", fontsize=HAZARD_AXIS_LABEL_FONTSIZE)
    ax.tick_params(labelsize=HAZARD_TICK_FONTSIZE)
    ax.legend(loc="best", fontsize=HAZARD_LEGEND_FONTSIZE)
    ax.set_title("Comparison raw path and simplified path",
                 fontsize=HAZARD_TITLE_FONTSIZE)
    plt.tight_layout()
else:
    print(f"\nNo {hazards_path.name} for this run (predates hazard export) -- skipping vectorized hazard plot")

plt.show()
