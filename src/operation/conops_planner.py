from dataclasses import dataclass
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import plotly.graph_objects as go

# categorical slots 1 (blue) and 2 (orange) -- validated adjacent-pair contrast
ACTOR_COLORS = {"rover": "#2a78d6", "earth": "#eb6834"}

# fixed row order for plot_timeline, matching the sequence tasks are built
# in within generate_waypoint_tasks -- not every task appears in every leg
# (turn/localization turn are conditional), so row order can't just follow
# first-appearance while scanning the whole timeline chronologically
TASK_ROW_ORDER = ["Walk", "turn", "localization turn", "Situational Picture", "Nav Downlink",
                   "GRAV measurement", "SPEC measurement",
                   "SciCAM measurement", "Payload Downlink", "nap", "earth_analysis", "uplink"]

# (key, display name, actor) in execution order for the sequential rover
# stop -- durations/power looked up as config_time[f"{key}_time"] /
# config_power[f"{key}_power"] (see robot_time_config.py / robot_power_config.py,
# both regenerated from user_data/config_robot/Input_Conops.xlsx by
# import_conops_excel.py). "drive" and "turn" are handled separately since
# their duration comes from the path geometry, not a fixed config value.
# "relay" (the sheet's "Payload data downlink", OPS-12-b) comes after the
# measurements it's actually downlinking, not right after the quick nav
# image downlink. SciCAM 360 measurement was removed -- it's no longer in
# the Excel.
STOP_TASK_SEQUENCE = [
    ("picture", "Situational Picture", "rover"),
    ("image_downlink", "Nav Downlink", "rover"),
    ("grav_measurement", "GRAV measurement", "rover"),
    ("spec_measurement", "SPEC measurement", "rover"),
    ("scicam_measurement", "SciCAM measurement", "rover"),
    ("relay", "Payload Downlink", "rover"),
    ("nap", "nap", "rover"),
]

# Alternate per-stop sequence for special waypoints (pit_ops only -- see
# path_analyzer.build_drive_segments' special_trigger_idxs and
# mission_ops_config.phase_params). Independently defined from
# STOP_TASK_SEQUENCE -- free to add/remove/reorder tasks -- but must still
# contain exactly one "image_downlink" entry (generate_waypoint_tasks hooks
# earth_analysis's start off it regardless of which sequence is active).
# "picture"/"spec_measurement" repeat counts come from phase_params[1]'s
# special_num_pictures/special_num_spec_measurements, not from how many
# times they're listed here.
SPECIAL_STOP_TASK_SEQUENCE = [
    ("picture", "Situational Picture", "rover"),
    ("image_downlink", "Nav Downlink", "rover"),
    ("spec_measurement", "SPEC measurement", "rover"),
    ("grav_measurement", "GRAV measurement", "rover"),
    ("scicam_measurement", "SciCAM measurement", "rover"),
    ("relay", "Payload Downlink", "rover"),
    ("nap", "nap", "rover"),
]


@dataclass
class Task:
    name: str
    actor: str
    start: float
    end: float
    phase: int = 0
    power_w: float = 0.0

def generate_waypoint_tasks(start_time, travel_time, turn_time, config_time, config_power,
                             phase=0, localization=False, special=False, repeat_counts=None):
    """
    Build one stop's worth of tasks: the drive/walk leg, an optional turn
    (reorientation for the next leg's heading), an optional localization
    turn, then the fixed sequence (STOP_TASK_SEQUENCE, or
    SPECIAL_STOP_TASK_SEQUENCE if special=True), with earth_analysis/uplink
    running in parallel on the ground side. Durations for turn/drive come
    from the path geometry (turn_time/travel_time); every other task's
    duration and power draw are looked up by key from config_time/config_power.

    repeat_counts: optional {key: N} -- append that task N times in a row
    instead of once (e.g. {"picture": 5} for 5 Situational Picture tasks).
    Keys not listed default to 1. Never target "image_downlink" with a
    repeat count -- its end time is what earth_analysis's start is hooked
    to below, and the check re-fires on every repeat, silently jumping to
    the last repeat's end.
    """
    tasks = []
    t = start_time
    repeat_counts = repeat_counts or {}

    def append_task(key, display, actor, duration):
        nonlocal t
        power = config_power.get(f"{key}_power", 0.0)
        tasks.append(Task(display, actor, t, t + duration, phase=phase, power_w=power))
        t += duration

    # Drive
    append_task("drive", "Walk", "rover", travel_time)

    # Turn / reorientation (direction changes encountered while driving this leg)
    if turn_time > 0:
        append_task("turn", "turn", "rover", turn_time)

    # Localization turn (forced every Nth measurement, see
    # mission_ops_config.localization_interval_measurements -- snapped onto
    # this stop rather than creating a separate one). Same physical
    # maneuver as Spot Turn (a stationary rover turn), so it reuses that
    # task's time/power rather than its own config value.
    if localization and config_time.get("turn_time", 0) > 0:
        append_task("turn", "localization turn", "rover", config_time["turn_time"])

    sequence = SPECIAL_STOP_TASK_SEQUENCE if special else STOP_TASK_SEQUENCE
    sequence_name = "SPECIAL_STOP_TASK_SEQUENCE" if special else "STOP_TASK_SEQUENCE"

    image_downlink_end = None
    for key, display, actor in sequence:
        for _ in range(repeat_counts.get(key, 1)):
            append_task(key, display, actor, config_time[f"{key}_time"])
            if key == "image_downlink":
                image_downlink_end = t
    assert image_downlink_end is not None, f'"image_downlink" must be in {sequence_name}'
    nap_end = t

    # Parallel activities. earth_analysis starts once the nav image has
    # finished downlinking -- Earth can start working from that quick
    # picture while the rover is still out taking measurements and
    # downlinking the (much larger) payload data.
    earth_analysis_time = config_time["earth_analysis_time"]
    earth_analysis_end = image_downlink_end + earth_analysis_time
    tasks.append(Task("earth_analysis", "earth", image_downlink_end, earth_analysis_end,
                       phase=phase, power_w=config_power.get("earth_analysis_power", 0.0)))

    # Uplink (must finish before next drive, i.e. after both earth_analysis
    # and the rover's own on-board sequence/nap have completed)
    uplink_time = config_time["uplink_time"]
    uplink_start = max(earth_analysis_end, nap_end)
    tasks.append(Task("uplink", "earth", uplink_start, uplink_start + uplink_time,
                       phase=phase, power_w=config_power.get("uplink_power", 0.0)))

    return tasks, uplink_start + uplink_time


def build_mission(segments, velocity, config_time, config_power_robot, localization_interval=None,
                   phase_params=None):
    """
    segments: list of {"distance": meters, "turns": count}, one per drive leg
    between measurement/downlink stops (see path_analyzer.build_drive_segments).

    localization_interval: force a "localization turn" every Nth measurement
    (each segment generates exactly one stop, so this is equivalently every
    Nth segment/stop). None (default) never forces one.

    phase_params: optional {phase: {"num_pictures": N, "special_num_pictures":
    N, "special_num_spec_measurements": N, ...}} (see mission_ops_config.py).
    Drives the picture-count-per-stop for ordinary stops (per phase) and, for
    segments tagged "special" by build_drive_segments, the picture/SPEC
    counts for SPECIAL_STOP_TASK_SEQUENCE instead. None (default) keeps
    today's behavior: exactly 1 picture per stop, never special.
    """

    timeline = []
    t = 0
    measurement_count = 0

    for seg in segments:

        travel_time = seg["distance"] / velocity
        turn_time = seg["turns"] * config_time.get("turn_time", 0)

        measurement_count += 1
        localization = bool(localization_interval) and measurement_count % localization_interval == 0

        phase = seg.get("phase", 0)
        special = seg.get("special", False)
        params = (phase_params or {}).get(phase, {})
        if special:
            repeat_counts = {"picture": params.get("special_num_pictures", 1),
                              "spec_measurement": params.get("special_num_spec_measurements", 1)}
        else:
            repeat_counts = {"picture": params.get("num_pictures", 1)}

        tasks, t = generate_waypoint_tasks(
            t, travel_time, turn_time, config_time, config_power_robot,
            phase=phase, localization=localization, special=special, repeat_counts=repeat_counts
        )

        timeline.extend(tasks)

    return timeline




def plot_timeline2(tasks):

    lanes = {
        #"rover": 10,
        #"earth": 20
        "drive": 0,
        "picture":1,
        "relay":2,
        "earth_analysis":3,
        "measurement":4
    }

    fig, ax = plt.subplots(figsize=(12,4))
    print (tasks)

    for task in tasks:
        ax.broken_barh(
            [(task.start, task.end-task.start)],
            (lanes[task.name], 5)
        )

    ax.set_yticks([0,5])
    ax.set_yticklabels(["rover", "earth"])
    ax.set_xlabel("Time")

    plt.show()

def plot_timeline(tasks, show=True, velocity=None, phase_names=None):
    """
    velocity: rover drive speed [m/s]. If given, each "drive" bar is
    annotated with the distance covered in that leg (distance = velocity *
    duration) and the axes title shows the cumulative distance driven.

    phase_names: names for task.phase 0, 1, ... (see marius_hills_mission.
    run_mission_planning). If given, a faint dashed line marks each phase
    change, labeled with the phase it's entering.
    """
    # row order follows TASK_ROW_ORDER (names not on that list, if any,
    # fall back to first-appearance order at the bottom)
    present = dict.fromkeys(task.name for task in tasks)
    names = [n for n in TASK_ROW_ORDER if n in present]
    names += [n for n in present if n not in names]

    # map each name to a y position
    y_map = {name: i for i, name in enumerate(names)}

    fig, ax = plt.subplots(figsize=(14,5))

    cumulative_distance_m = 0.0
    for task in tasks:
        unit = 60 #transform from seconds to minutes
        start_time = task.start / 60
        duration_time = (task.end - task.start) / 60

        y = y_map[task.name]
        color = ACTOR_COLORS.get(task.actor, "#898781")

        ax.broken_barh(
            [(start_time, duration_time)],
            (y - 0.4, 0.5),
            facecolor=color, edgecolor=color
        )

        if velocity is not None and task.name == "Walk":
            leg_distance_m = velocity * (task.end - task.start)
            cumulative_distance_m += leg_distance_m
            ax.annotate(f"{leg_distance_m:.0f} m", (start_time + duration_time / 2, y),
                        ha="center", va="center", fontsize=12)

    ax.set_yticks(list(y_map.values()))
    ax.set_yticklabels(list(y_map.keys()))
    ax.set_xlabel("Time [min]")
    if velocity is not None:
        ax.set_title(f"Total distance driven: {cumulative_distance_m:.0f} m")

    # phase-change markers: a faint dashed line at the midpoint between the
    # last task of one phase and the first task of the next (tasks aren't
    # strictly ordered by phase at the boundary -- earth-side tasks of the
    # last phase-0 stop can end after the first phase-1 stop's rover tasks
    # start -- so the midpoint keeps the line out of either stop's bars)
    phases_present = sorted(set(task.phase for task in tasks))
    for p0, p1 in zip(phases_present[:-1], phases_present[1:]):
        p0_end = max(task.end for task in tasks if task.phase == p0) / 60
        p1_start = min(task.start for task in tasks if task.phase == p1) / 60
        boundary = (p0_end + p1_start) / 2
        ax.axvline(boundary, color="#898781", linestyle="--", linewidth=1, alpha=0.6, zorder=0)
        if phase_names and p1 < len(phase_names):
            ax.text(boundary, ax.get_ylim()[1], f" {phase_names[p1]}",
                     ha="left", va="bottom", fontsize=12, color="#52514e")

    legend_handles = [Line2D([0], [0], color=c, lw=6, label=actor)
                       for actor, c in ACTOR_COLORS.items()]
    ax.legend(handles=legend_handles, loc="upper right", frameon=False)

    plt.tight_layout()
    if show:
        plt.show()

    return fig


def plot_timeline_interactive(tasks, velocity=None, phase_names=None):
    """
    Interactive Plotly equivalent of plot_timeline, saved as a self-contained
    .html file (see marius_hills_mission.run_mission_planning). Opens in a
    browser with the same scroll-to-zoom/drag-to-pan behavior as the live
    matplotlib window -- unlike a static PNG/PDF, the axis re-renders at
    whatever zoom level you're at, so it never runs out of legible ticks.
    """
    present = dict.fromkeys(task.name for task in tasks)
    names = [n for n in TASK_ROW_ORDER if n in present]
    names += [n for n in present if n not in names]

    fig = go.Figure()

    cumulative_distance_m = 0.0
    for actor, color in ACTOR_COLORS.items():
        actor_tasks = [t for t in tasks if t.actor == actor]
        if not actor_tasks:
            continue

        starts_min = [t.start / 60 for t in actor_tasks]
        durations_min = [(t.end - t.start) / 60 for t in actor_tasks]
        hover = [f"{t.name}<br>start: {t.start / 60:.1f} min<br>"
                 f"duration: {(t.end - t.start) / 60:.1f} min<br>power: {t.power_w:.1f} W"
                 for t in actor_tasks]

        fig.add_trace(go.Bar(
            x=durations_min, y=[t.name for t in actor_tasks], base=starts_min,
            orientation="h", marker_color=color, name=actor,
            hovertext=hover, hoverinfo="text", width=0.5,
        ))

        if velocity is not None:
            for t in actor_tasks:
                if t.name != "Walk":
                    continue
                leg_distance_m = velocity * (t.end - t.start)
                cumulative_distance_m += leg_distance_m
                fig.add_annotation(
                    x=(t.start + (t.end - t.start) / 2) / 60, y=t.name,
                    text=f"{leg_distance_m:.0f} m", showarrow=False, font=dict(size=14))

    phases_present = sorted(set(task.phase for task in tasks))
    for p0, p1 in zip(phases_present[:-1], phases_present[1:]):
        p0_end = max(task.end for task in tasks if task.phase == p0) / 60
        p1_start = min(task.start for task in tasks if task.phase == p1) / 60
        boundary = (p0_end + p1_start) / 2
        fig.add_vline(x=boundary, line_dash="dash", line_color="#898781", opacity=0.6)
        if phase_names and p1 < len(phase_names):
            fig.add_annotation(x=boundary, y=1.0, yref="paper", yanchor="bottom",
                                text=phase_names[p1], showarrow=False,
                                font=dict(size=17, color="#52514e"), xanchor="left")

    fig.update_layout(
        barmode="overlay",
        yaxis=dict(categoryorder="array", categoryarray=names),
        xaxis_title="Time [min]",
        title=f"Total distance driven: {cumulative_distance_m:.0f} m" if velocity is not None else None,
        legend_title_text="Actor",
        height=600, width=1400,
        margin=dict(t=80),
    )

    return fig



