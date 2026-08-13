from dataclasses import dataclass
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import plotly.graph_objects as go

# categorical slots 1 (blue) and 2 (orange) -- validated adjacent-pair contrast
ACTOR_COLORS = {"rover": "#2a78d6", "earth": "#eb6834"}

# fixed row order for plot_timeline, matching the sequence tasks are built
# in within generate_waypoint_tasks -- not every task appears in every leg
# (turn is conditional), so row order can't just follow first-appearance
# while scanning the whole timeline chronologically. "localization turn"
# sits right next to "SciCAM 360 measurement": they're mutually-exclusive
# labels for the same sequence slot, never both present on the same stop.
TASK_ROW_ORDER = ["turn", "drive", "Situational Picture", "Nav Downlink",
                   "SciCAM 360 measurement", "localization turn", "GRAV measurement", "SPEC measurement",
                   "SciCAM measurement", "Payload Downlink", "nap", "earth_analysis", "uplink"]

# (key, display name, actor) in execution order for the sequential rover
# stop -- durations/power looked up as config_time[f"{key}_time"] /
# config_power[f"{key}_power"] (see robot_time_config.py / robot_power_config.py,
# both regenerated from user_data/config_robot/Input_Conops.xlsx by
# import_conops_excel.py). "drive" and "turn" are handled separately since
# their duration comes from the path geometry, not a fixed config value.
# "relay" (the sheet's "Payload data downlink", OPS-12-b) comes after the
# measurements it's actually downlinking, not right after the quick nav
# image downlink.
STOP_TASK_SEQUENCE = [
    ("picture", "Situational Picture", "rover"),
    ("image_downlink", "Nav Downlink", "rover"),
    ("scicam_360", "SciCAM 360 measurement", "rover"),
    ("grav_measurement", "GRAV measurement", "rover"),
    ("spec_measurement", "SPEC measurement", "rover"),
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
                             phase=0, localization=False):
    """
    Build one stop's worth of tasks: an optional turn, the drive leg, then
    the fixed STOP_TASK_SEQUENCE, with earth_analysis/uplink running in
    parallel on the ground side. Durations for turn/drive come from the
    path geometry (turn_time/travel_time); every other task's duration and
    power draw are looked up by key from config_time/config_power.

    localization: on a forced-localization stop (every Nth measurement,
    see mission_ops_config.localization_interval_measurements), the
    scicam_360 slot is relabeled "localization turn" instead of "SciCAM
    360 measurement" -- same maneuver (a 360-degree turn while imaging),
    same time/power, just serving double duty on this stop. Not an extra
    task: that would duplicate the same physical activity twice.
    """
    tasks = []
    t = start_time

    def append_task(key, display, actor, duration):
        nonlocal t
        power = config_power.get(f"{key}_power", 0.0)
        tasks.append(Task(display, actor, t, t + duration, phase=phase, power_w=power))
        t += duration

    # Turn / reorientation (direction changes encountered while driving this leg)
    if turn_time > 0:
        append_task("turn", "turn", "rover", turn_time)

    # Drive
    append_task("drive", "drive", "rover", travel_time)

    relay_start = None
    for key, display, actor in STOP_TASK_SEQUENCE:
        if key == "scicam_360" and localization:
            display = "localization turn"
        if key == "relay":
            relay_start = t
        append_task(key, display, actor, config_time[f"{key}_time"])
    assert relay_start is not None, '"relay" must be in STOP_TASK_SEQUENCE'
    nap_end = t

    # Parallel activities. earth_analysis starts as soon as the payload
    # downlink begins (Earth can start working on data as it streams in),
    # not once the whole downlink has finished.
    earth_analysis_time = config_time["earth_analysis_time"]
    earth_analysis_end = relay_start + earth_analysis_time
    tasks.append(Task("earth_analysis", "earth", relay_start, earth_analysis_end,
                       phase=phase, power_w=config_power.get("earth_analysis_power", 0.0)))

    # Uplink (must finish before next drive, i.e. after both earth_analysis
    # and the rover's own on-board sequence/nap have completed)
    uplink_time = config_time["uplink_time"]
    uplink_start = max(earth_analysis_end, nap_end)
    tasks.append(Task("uplink", "earth", uplink_start, uplink_start + uplink_time,
                       phase=phase, power_w=config_power.get("uplink_power", 0.0)))

    return tasks, uplink_start + uplink_time


def build_mission(segments, velocity, config_time, config_power_robot, localization_interval=None):
    """
    segments: list of {"distance": meters, "turns": count}, one per drive leg
    between measurement/downlink stops (see path_analyzer.build_drive_segments).

    localization_interval: force a "localization turn" every Nth measurement
    (each segment generates exactly one stop, so this is equivalently every
    Nth segment/stop). None (default) never forces one.
    """

    timeline = []
    t = 0
    measurement_count = 0

    for seg in segments:

        travel_time = seg["distance"] / velocity
        turn_time = seg["turns"] * config_time.get("turn_time", 0)

        measurement_count += 1
        localization = bool(localization_interval) and measurement_count % localization_interval == 0

        tasks, t = generate_waypoint_tasks(
            t, travel_time, turn_time, config_time, config_power_robot,
            phase=seg.get("phase", 0), localization=localization
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

        if velocity is not None and task.name == "drive":
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
                if t.name != "drive":
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



