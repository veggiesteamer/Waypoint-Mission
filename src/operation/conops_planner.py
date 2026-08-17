from dataclasses import dataclass
import math
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import plotly.graph_objects as go

# categorical slots 1 (blue) and 2 (orange) -- validated adjacent-pair contrast
ACTOR_COLORS = {"rover": "#2a78d6", "earth": "#eb6834"}

# fixed row order for plot_timeline, matching the sequence tasks are built
# in within generate_waypoint_tasks -- not every task appears in every leg
# (turn/localization turn are conditional), so row order can't just follow
# first-appearance while scanning the whole timeline chronologically
TASK_ROW_ORDER = ["Walk", "turn", "localization turn", "Locomotion Test",
                   "Situational Picture", "Nav Downlink",
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

# key -> (display name, actor), for every task STOP_TASK_SEQUENCE knows
# about -- also the vocabulary special waypoints can draw on (see
# mission_ops_config.phase_params[1]["special_sequence"]).
TASK_CATALOG = {key: (display, actor) for key, display, actor in STOP_TASK_SEQUENCE}

# key -> display name, for every locomotion-experiment task defined in
# user_data/config_robot/locomotion_experiment_config.py (time/power come
# from there, not the Excel-derived robot_time_config/robot_power_config --
# see generate_locomotion_experiment_tasks). Add an entry here whenever a
# new key is added to that config file.
LOCOMOTION_TASK_CATALOG = {
    "locomotion_test": "Locomotion Test",
}


def generate_locomotion_experiment_tasks(start_time, sequence, locomotion_config, phase=0):
    """
    Build a standalone block of locomotion-experiment tasks (see
    mission_ops_config.locomotion_experiment), inserted between regular
    stops rather than as part of one. Each key in `sequence` (from
    LOCOMOTION_TASK_CATALOG) becomes its own extra drive: duration_s/energy_wh
    at locomotion_config[key] set the block directly (power_w is derived,
    not configured), run in place in the timeline -- it costs time/power but
    doesn't move the rover's mapped position, since this simulator doesn't
    model real 2D test-pattern kinematics. List a key more than once in
    `sequence` to repeat it.
    """
    tasks = []
    t = start_time
    for key in sequence:
        display = LOCOMOTION_TASK_CATALOG[key]
        params = locomotion_config[key]
        duration = params["duration_s"]
        power_w = params["energy_wh"] * 3600 / duration
        tasks.append(Task(display, "rover", t, t + duration, phase=phase, power_w=power_w))
        t += duration
    return tasks, t


def nap_duration_s(t_seconds, target_average_s, nap_curve, mission_start_lunar_hour=0.0,
                    lunar_day_hours=708.7):
    """
    Nap duration (s) as a function of elapsed mission time, following a
    lunar-day thermal/power cycle: longer in the morning (recharging after
    lunar night) and around local noon (extra cooldown from peak solar
    heating), shorter in between (see mission_ops_config.py: lunar_day_hours,
    mission_start_lunar_hour, nap_curve).

    nap_curve: {"morning_peak_hour", "morning_peak_amplitude_s",
    "morning_peak_width_h", "noon_peak_hour", "noon_peak_amplitude_s",
    "noon_peak_width_h"} -- each peak is a Gaussian bump (width = standard
    deviation, in hours) added on top of a baseline. The baseline is solved
    for so the curve's average over one full lunar day comes out to exactly
    target_average_s regardless of the amplitudes/widths chosen: a Gaussian
    bump's mean over one full period is amplitude * width * sqrt(2*pi) /
    lunar_day_hours (exact in the limit width << period, an excellent
    approximation here since the widths are tens of hours against a
    ~708.7 h period). That's also why "in between" comes out shorter than
    average for free -- the baseline sits below target_average_s to make
    room for the bumps.
    """
    lunar_hour = (mission_start_lunar_hour + t_seconds / 3600) % lunar_day_hours

    def bump(center_h, amplitude_s, width_h):
        return amplitude_s * math.exp(-((lunar_hour - center_h) ** 2) / (2 * width_h ** 2))

    def bump_mean(amplitude_s, width_h):
        return amplitude_s * width_h * math.sqrt(2 * math.pi) / lunar_day_hours

    morning = bump(nap_curve["morning_peak_hour"], nap_curve["morning_peak_amplitude_s"],
                    nap_curve["morning_peak_width_h"])
    noon = bump(nap_curve["noon_peak_hour"], nap_curve["noon_peak_amplitude_s"],
                nap_curve["noon_peak_width_h"])
    correction = (bump_mean(nap_curve["morning_peak_amplitude_s"], nap_curve["morning_peak_width_h"]) +
                  bump_mean(nap_curve["noon_peak_amplitude_s"], nap_curve["noon_peak_width_h"]))
    baseline = target_average_s - correction

    return max(baseline + morning + noon, 0.0)


@dataclass
class Task:
    name: str
    actor: str
    start: float
    end: float
    phase: int = 0
    power_w: float = 0.0

def generate_waypoint_tasks(start_time, travel_time, turn_time, config_time, config_power,
                             phase=0, localization=False, special=False, repeat_counts=None,
                             special_sequence=None, nap_curve=None, mission_start_lunar_hour=0.0,
                             lunar_day_hours=708.7, skip_tasks=None):
    """
    Build one stop's worth of tasks: the drive/walk leg, an optional turn
    (reorientation for the next leg's heading), an optional localization
    turn, then the fixed sequence (STOP_TASK_SEQUENCE, or special_sequence
    if special=True), with earth_analysis/uplink running in parallel on the
    ground side. Durations for turn/drive come from the path geometry
    (turn_time/travel_time); every other task's duration and power draw are
    looked up by key from config_time/config_power.

    repeat_counts: optional {key: N} -- append that task N times in a row
    instead of once (e.g. {"picture": 5} for 5 Situational Picture tasks).
    Keys not listed default to 1. Only applies to the ordinary (non-special)
    sequence -- never target "image_downlink" with a repeat count either
    way, since its end time is what earth_analysis's start is hooked to
    below, and the check re-fires on every repeat, silently jumping to the
    last repeat's end.

    special_sequence: ordered list of TASK_CATALOG keys, run instead of
    STOP_TASK_SEQUENCE when special=True (see mission_ops_config.
    phase_params[1]["special_sequence"]) -- freely repeat/reorder/omit
    tasks by listing keys however you want; repeats are literal (list a key
    twice for it to run twice) rather than going through repeat_counts.
    Must contain "image_downlink" exactly once.

    nap_curve/mission_start_lunar_hour/lunar_day_hours: if nap_curve is
    given (see mission_ops_config.py), every "nap" task's duration is
    computed from nap_duration_s at the moment it starts, instead of the
    flat config_time["nap_time"]. None (default) keeps the flat value.

    skip_tasks: optional set of TASK_CATALOG keys to omit entirely from the
    ordinary (non-special) sequence at this stop (see mission_ops_config.
    phase_params[<phase>]["task_intervals"] -- build_mission works out, per
    stop, which tasks fall outside their configured "every Nth stop"
    interval). "image_downlink" is always exempt regardless of what's
    passed in -- it anchors earth_analysis's start time and must occur
    exactly once per stop. Ignored when special=True (special_sequence is
    already a complete, self-contained plan).
    """
    tasks = []
    t = start_time
    repeat_counts = repeat_counts or {}
    skip_tasks = (skip_tasks or set()) - {"image_downlink"}

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

    if special:
        assert special_sequence, ('special=True requires mission_ops_config.'
                                   'phase_params[<phase>]["special_sequence"] to be set')
        sequence = [(key,) + TASK_CATALOG[key] for key in special_sequence]
        sequence_name = "special_sequence"
    else:
        sequence = STOP_TASK_SEQUENCE
        sequence_name = "STOP_TASK_SEQUENCE"

    image_downlink_end = None
    for key, display, actor in sequence:
        if not special and key in skip_tasks:
            continue
        n = 1 if special else repeat_counts.get(key, 1)
        for _ in range(n):
            if key == "nap" and nap_curve is not None:
                duration = nap_duration_s(t, config_time["nap_time"], nap_curve,
                                           mission_start_lunar_hour, lunar_day_hours)
            else:
                duration = config_time[f"{key}_time"]
            append_task(key, display, actor, duration)
            if key == "image_downlink":
                image_downlink_end = t
    assert image_downlink_end is not None, f'"image_downlink" must appear exactly once in {sequence_name}'
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
                   phase_params=None, nap_curve=None, mission_start_lunar_hour=0.0,
                   lunar_day_hours=708.7, locomotion_experiment=None, locomotion_config=None):
    """
    segments: list of {"distance": meters, "turns": count}, one per drive leg
    between measurement/downlink stops (see path_analyzer.build_drive_segments).

    localization_interval: force a "localization turn" every Nth measurement
    (each segment generates exactly one stop, so this is equivalently every
    Nth segment/stop). None (default) never forces one.

    phase_params: optional {phase: {"num_pictures": N, "special_sequence":
    [...], "task_intervals": {...}}} (see mission_ops_config.py).
    num_pictures drives the picture count per ordinary stop in that phase.
    special_sequence (a list of TASK_CATALOG keys, see
    generate_waypoint_tasks) is required for any phase that can have
    segments tagged "special" by build_drive_segments -- it replaces
    STOP_TASK_SEQUENCE entirely at that stop. task_intervals ({key: N} for
    grav_measurement/spec_measurement/scicam_measurement/relay) makes that
    task run only every Nth measurement/stop instead of every stop -- same
    "every Nth" cadence as localization_interval below, just per task; keys
    not listed default to every stop (interval 1), and this never applies
    to special stops. None (default) keeps today's behavior: exactly 1
    picture per stop, every task every stop, never special.

    nap_curve/mission_start_lunar_hour/lunar_day_hours: see
    generate_waypoint_tasks/nap_duration_s. nap_curve=None (default) keeps
    every nap at the flat config_time["nap_time"].

    locomotion_experiment/locomotion_config: locomotion_experiment (see
    mission_ops_config.py) is {"trigger_after_special_occurrence": [1, 2,
    ...], "sequence": [...]} -- a standalone block of locomotion_config
    tasks (see generate_locomotion_experiment_tasks) is inserted right
    after the Nth special-waypoint stop finishes, for each N listed (1 =
    after the 1st special stop, 2 = after the 2nd, etc.). Either argument
    None (default) never inserts one.
    """

    timeline = []
    t = 0
    measurement_count = 0
    special_occurrence_count = 0
    locomotion_trigger_occurrences = set(
        (locomotion_experiment or {}).get("trigger_after_special_occurrence", []))

    for seg in segments:

        travel_time = seg["distance"] / velocity
        turn_time = seg["turns"] * config_time.get("turn_time", 0)

        measurement_count += 1
        localization = bool(localization_interval) and measurement_count % localization_interval == 0

        phase = seg.get("phase", 0)
        special = seg.get("special", False)
        params = (phase_params or {}).get(phase, {})
        repeat_counts = {"picture": params.get("num_pictures", 1)}
        special_sequence = params.get("special_sequence") if special else None

        task_intervals = params.get("task_intervals", {})
        skip_tasks = {key for key, interval in task_intervals.items()
                      if interval and measurement_count % interval != 0}

        tasks, t = generate_waypoint_tasks(
            t, travel_time, turn_time, config_time, config_power_robot,
            phase=phase, localization=localization, special=special,
            repeat_counts=repeat_counts, special_sequence=special_sequence,
            nap_curve=nap_curve, mission_start_lunar_hour=mission_start_lunar_hour,
            lunar_day_hours=lunar_day_hours, skip_tasks=skip_tasks
        )

        timeline.extend(tasks)

        if special:
            special_occurrence_count += 1
            if (locomotion_experiment and locomotion_config
                    and special_occurrence_count in locomotion_trigger_occurrences):
                loco_tasks, t = generate_locomotion_experiment_tasks(
                    t, locomotion_experiment["sequence"], locomotion_config, phase=phase)
                timeline.extend(loco_tasks)

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

    # thin, faint separator at the start of every stop (marked by "Walk"),
    # so consecutive stops are easy to tell apart at a glance
    for task in tasks:
        if task.name == "Walk":
            ax.axvline(task.start / 60, color="#c3c2b7", linewidth=0.6, alpha=0.4, zorder=0)

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

    # thin, faint separator at the start of every stop (marked by "Walk"),
    # so consecutive stops are easy to tell apart at a glance
    for t in tasks:
        if t.name == "Walk":
            fig.add_vline(x=t.start / 60, line_color="#c3c2b7", line_width=0.6,
                           opacity=0.4, layer="below")

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



