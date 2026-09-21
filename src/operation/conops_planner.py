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
TASK_ROW_ORDER = ["Egress", "Commissioning", "Walk", "turn", "standing", "localization turn", "Locomotion Test",
                   "Situational Picture", "Nav Downlink",
                   "GRAV measurement",
                   "SciCAM measurement", "Payload Downlink", "nap", "SPEC warmup", "SPEC measurement",
                   "earth_analysis", "uplink"]

# (key, display name, actor) in execution order for the sequential rover
# stop -- durations/power looked up as config_time[f"{key}_time"] /
# config_power[f"{key}_power"] (see robot_time_config.py / robot_power_config.py,
# both regenerated from user_data/config_robot/Input_Conops.xlsx by
# import_conops_excel.py). "drive" and "turn" are handled separately since
# their duration comes from the path geometry, not a fixed config value.
# "relay" (the sheet's "Payload data downlink", OPS-12-b) comes after the
# measurements it's actually downlinking, not right after the quick nav
# image downlink. SciCAM 360 measurement was removed -- it's no longer in
# the Excel. SPEC measurement runs last, after nap, preceded by its own
# warmup -- the instrument needs a relatively long warmup before an
# accurate reading, so warmup+measurement are deliberately the tail of the
# stop rather than interleaved with the other measurements.
STOP_TASK_SEQUENCE = [
    ("picture", "Situational Picture", "rover"),
    ("image_downlink", "Nav Downlink", "rover"),
    ("grav_measurement", "GRAV measurement", "rover"),
    ("scicam_measurement", "SciCAM measurement", "rover"),
    ("relay", "Payload Downlink", "rover"),
    ("nap", "nap", "rover"),
    ("spec_warmup", "SPEC warmup", "rover"),
    ("spec_measurement", "SPEC measurement", "rover"),
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

# key -> (display name, actor) for tasks usable *only* inside an explicit
# special_sequence list (see mission_ops_config.phase_params[<phase>]
# ["special_sequence"]) -- unlike TASK_CATALOG's keys, these are never
# automatically part of the ordinary per-stop STOP_TASK_SEQUENCE, since
# generate_waypoint_tasks only consults this catalog when special=True.
# "turn" here is a re-pointing maneuver during a special stop (e.g. Pit
# Wall Inspection) -- a separate occurrence from the automatic per-leg
# reorientation turn (path-geometry-driven, added unconditionally before
# either sequence starts) and from "localization turn" (periodic, see
# localization_interval_distance_m/localization_sequence below), but
# deliberately sharing the same key/display name and the same
# config_time["turn_time"]/config_power["turn_power"] ("Spot Turn") values
# as the automatic one -- same physical maneuver, different context/
# repeat count, not worth a fourth distinct name on the timeline.
SPECIAL_ONLY_TASK_CATALOG = {
    "turn": ("turn", "rover"),
}

# What generate_waypoint_tasks actually looks a special_sequence key up
# in -- ordinary STOP_TASK_SEQUENCE keys remain usable there too (e.g.
# "spec_measurement"), on top of SPECIAL_ONLY_TASK_CATALOG's keys.
TASK_CATALOG_FOR_SPECIAL_SEQUENCE = {**TASK_CATALOG, **SPECIAL_ONLY_TASK_CATALOG}


def batched_sequence(key, batch_size, total, interleave=None):
    """
    Build a flat list of `total` `key` entries (a TASK_CATALOG key, e.g.
    "spec_measurement"), `batch_size` at a time, with `interleave` (a list
    of keys, e.g. ["scicam_measurement"]) inserted between consecutive
    batches -- never after the last one. The final batch is smaller than
    batch_size if `total` isn't evenly divisible by it (e.g. total=62,
    batch_size=5 -> twelve batches of 5, one final batch of 2).

    For splicing into a phase_params[phase]["special_sequence"] list (see
    mission_ops_config.py) without hand-writing/maintaining every repeat:

        "special_sequence": [
            "picture", "image_downlink", "spec_warmup",
            *batched_sequence("spec_measurement", batch_size=5, total=60,
                               interleave=["scicam_measurement"]),
            "grav_measurement", "relay", "nap",
        ],
    """
    interleave = interleave or []
    sequence = []
    remaining = total
    first = True
    while remaining > 0:
        if not first:
            sequence.extend(interleave)
        n = min(batch_size, remaining)
        sequence.extend([key] * n)
        remaining -= n
        first = False
    return sequence


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
    # the source CONOPS's own Task ID (e.g. "OPS-10"), see
    # user_data/config_robot/robot_task_ids_config.py -- None for tasks the
    # sheet gives no ID for, and for tasks that aren't in the Excel CONOPS
    # at all (Egress, Commissioning, Locomotion Test)
    task_id: str = None

def generate_waypoint_tasks(start_time, travel_time, turn_time, config_time, config_power,
                             phase=0, localization=False, special=False, repeat_counts=None,
                             special_sequence=None, nap_curve=None, mission_start_lunar_hour=0.0,
                             lunar_day_hours=708.7, skip_tasks=None, nap_constant_s=None, task_ids=None,
                             localization_sequence=None, standing_enabled=True):
    """
    Build one stop's worth of tasks: the drive/walk leg, an optional turn
    (reorientation for the next leg's heading), a standing/settle task, an
    optional localization turn, then the fixed sequence (STOP_TASK_SEQUENCE,
    or special_sequence if special=True), with earth_analysis/uplink running
    in parallel on the ground side. Durations for turn/drive come from the
    path geometry (turn_time/travel_time); every other task's duration and
    power draw are looked up by key from config_time/config_power.

    standing (config_time["standing_time"]/config_power["standing_power"],
    "Standing in any pose (braking)"): lets the rover settle after driving
    (and any per-leg turn) before any image is taken. Fires once,
    unconditionally, right after the drive/turn above -- before the
    localization sequence if this stop has one, otherwise before the main/
    special sequence's own first task. On top of that, any *additional*
    "turn" encountered later in the stop (a localization_sequence turn, or
    a "turn" entry inside special_sequence) gets its own standing inserted
    right before the next non-turn task, so a run of repeated measurements/
    images taken back-to-back with no further turning between them (e.g.
    the SPEC/SciCAM batch in special_sequence) never gets a redundant one.
    Skipped entirely if config_time["standing_time"] is 0/unset, or if
    standing_enabled=False (see mission_ops_config.standing_enabled) --
    same "mission-wide behavior toggle in mission_ops_config.py" pattern as
    nap_mode/egress/commissioning/locomotion_experiment.

    repeat_counts: optional {key: N} -- append that task N times in a row
    instead of once (e.g. {"picture": 5} for 5 Situational Picture tasks).
    Keys not listed default to 1. Only applies to the ordinary (non-special)
    sequence -- never target "image_downlink" with a repeat count either
    way, since its end time is what earth_analysis's start is hooked to
    below, and the check re-fires on every repeat, silently jumping to the
    last repeat's end.

    special_sequence: ordered list of keys, run instead of STOP_TASK_SEQUENCE
    when special=True (see mission_ops_config.phase_params[1]
    ["special_sequence"]) -- freely repeat/reorder/omit tasks by listing
    keys however you want; repeats are literal (list a key twice for it to
    run twice) rather than going through repeat_counts. Must contain
    "image_downlink" exactly once. Each key is looked up in
    TASK_CATALOG_FOR_SPECIAL_SEQUENCE, so both ordinary STOP_TASK_SEQUENCE
    keys (e.g. "spec_measurement") and SPECIAL_ONLY_TASK_CATALOG keys
    (currently just "turn", a re-pointing maneuver -- see that dict's
    docstring) are valid. An entry can also be a dict instead of a plain
    key -- {"batch": key, "batch_size": N, "total": M, "interleave":
    [...]} -- expanded via batched_sequence(key, N, M, interleave) before
    lookup, for long repeated runs (e.g. 60 spectra in batches of 5)
    without hand-writing/maintaining the flat repeat list.

    localization_sequence: ordered list of TASK_CATALOG_FOR_SPECIAL_SEQUENCE
    keys run when localization=True, instead of a single turn (see
    mission_ops_config.localization_sequence) -- e.g. ["turn", "turn",
    "scicam_measurement"] for two re-pointing turns then a science image.
    "turn" entries specifically display as "localization turn" (distinct
    from the automatic per-leg turn and from special_sequence's own "turn"
    entries) but reuse the same turn_time/turn_power value; every other key
    uses its own normal display name/duration/power. None (default) is a
    single "turn", matching the original (pre-sequence) behavior. This
    doesn't change *how often* it happens -- that's still
    localization_interval_distance_m/localization -- only what runs each
    time it does.

    nap_curve/mission_start_lunar_hour/lunar_day_hours: if nap_curve is
    given (see mission_ops_config.py), every "nap" task's duration is
    computed from nap_duration_s at the moment it starts. Otherwise, if
    nap_constant_s is given, every nap is that many seconds flat. If
    neither is given, naps fall back to the flat config_time["nap_time"]
    (today's default behavior). nap_curve takes priority if both happen to
    be set.

    skip_tasks: optional set of TASK_CATALOG keys to omit entirely from the
    ordinary (non-special) sequence at this stop (see mission_ops_config.
    phase_params[<phase>]["task_intervals"] -- build_mission works out, per
    stop, which tasks fall outside their configured "every Nth stop"
    interval). "image_downlink" is always exempt regardless of what's
    passed in -- it anchors earth_analysis's start time and must occur
    exactly once per stop. Ignored when special=True (special_sequence is
    already a complete, self-contained plan).

    task_ids: optional {key: "OPS-xx" or None} (see
    user_data/config_robot/robot_task_ids_config.py) -- stamped onto each
    Task's task_id field for traceability back to the source CONOPS. None
    (default) leaves every task_id unset.
    """
    tasks = []
    t = start_time
    repeat_counts = repeat_counts or {}
    skip_tasks = (skip_tasks or set()) - {"image_downlink"}
    task_ids = task_ids or {}

    def append_task(key, display, actor, duration):
        nonlocal t
        power = config_power.get(f"{key}_power", 0.0)
        tasks.append(Task(display, actor, t, t + duration, phase=phase, power_w=power,
                           task_id=task_ids.get(key)))
        t += duration

    # Drive
    append_task("drive", "Walk", "rover", travel_time)

    # Turn / reorientation (direction changes encountered while driving this leg)
    if turn_time > 0:
        append_task("turn", "turn", "rover", turn_time)

    # Standing: let the rover settle/brake after driving (and any per-leg
    # turn) before taking any image -- fires once per stop, unconditionally
    # (even on a leg that needed no turn), covering both the upcoming nav
    # picture and, if this stop also runs a localization sequence, that
    # sequence's own science image. Gated on standing_enabled (see
    # mission_ops_config.standing_enabled) on top of the usual
    # duration-configured check, so it can be switched off mission-wide.
    standing_active = standing_enabled and config_time.get("standing_time", 0) > 0
    if standing_active:
        append_task("standing", "standing", "rover", config_time["standing_time"])

    # Extra settle after any *additional* turn beyond the one above --
    # localization_sequence's own re-pointing turns, or a "turn" entry
    # inside special_sequence -- inserted right before the next non-turn
    # task. Repeated measurements/images taken back-to-back with no further
    # turning in between (e.g. the SPEC/SciCAM batch in special_sequence)
    # never get a redundant standing this way.
    pending_turn_settle = False

    def append_with_turn_settle(key, display, actor, duration):
        nonlocal pending_turn_settle
        if key == "turn":
            append_task(key, display, actor, duration)
            pending_turn_settle = True
        else:
            if pending_turn_settle and standing_active:
                append_task("standing", "standing", "rover", config_time["standing_time"])
            append_task(key, display, actor, duration)
            pending_turn_settle = False

    # Localization sequence (forced roughly every N meters walked, see
    # mission_ops_config.localization_interval_distance_m -- snapped onto
    # this stop rather than creating a separate one). Each "turn" entry
    # reuses Spot Turn's time/power like the automatic per-leg turn does,
    # but displays as "localization turn" to keep it distinct on the
    # timeline; every other key (e.g. "scicam_measurement") uses its own
    # normal display name/duration/power. Defaults to a single turn,
    # matching the original (pre-sequence) behavior.
    if localization:
        for key in (localization_sequence or ["turn"]):
            display, actor = TASK_CATALOG_FOR_SPECIAL_SEQUENCE[key]
            if key == "turn":
                display = "localization turn"
                duration = config_time.get("turn_time", 0)
                if duration <= 0:
                    continue
            else:
                duration = config_time[f"{key}_time"]
            append_with_turn_settle(key, display, actor, duration)

    if special:
        assert special_sequence, ('special=True requires mission_ops_config.'
                                   'phase_params[<phase>]["special_sequence"] to be set')
        # a plain dict entry is a batch marker (see batched_sequence) instead
        # of a literal key -- expanded here so mission_ops_config.py can stay
        # plain data (batch_size/total as numbers) without importing
        # this module to call batched_sequence itself
        expanded_sequence = []
        for item in special_sequence:
            if isinstance(item, dict):
                expanded_sequence.extend(batched_sequence(
                    item["batch"], item["batch_size"], item["total"], item.get("interleave")))
            else:
                expanded_sequence.append(item)
        sequence = [(key,) + TASK_CATALOG_FOR_SPECIAL_SEQUENCE[key] for key in expanded_sequence]
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
            elif key == "nap" and nap_constant_s is not None:
                duration = nap_constant_s
            else:
                duration = config_time[f"{key}_time"]
            append_with_turn_settle(key, display, actor, duration)
            if key == "image_downlink":
                image_downlink_end = t
    assert image_downlink_end is not None, f'"image_downlink" must appear exactly once in {sequence_name}'
    # whatever ends the sequence now -- SPEC warmup/measurement, not nap,
    # since those run last (see STOP_TASK_SEQUENCE)
    rover_sequence_end = t

    # Parallel activities. earth_analysis starts once the nav image has
    # finished downlinking -- Earth can start working from that quick
    # picture while the rover is still out taking measurements and
    # downlinking the (much larger) payload data.
    earth_analysis_time = config_time["earth_analysis_time"]
    earth_analysis_end = image_downlink_end + earth_analysis_time
    tasks.append(Task("earth_analysis", "earth", image_downlink_end, earth_analysis_end,
                       phase=phase, power_w=config_power.get("earth_analysis_power", 0.0),
                       task_id=task_ids.get("earth_analysis")))

    # Uplink (must finish before next drive, i.e. after both earth_analysis
    # and the rover's full on-board sequence -- now ending with SPEC
    # warmup/measurement, not nap -- have completed)
    uplink_time = config_time["uplink_time"]
    uplink_start = max(earth_analysis_end, rover_sequence_end)
    tasks.append(Task("uplink", "earth", uplink_start, uplink_start + uplink_time,
                       phase=phase, power_w=config_power.get("uplink_power", 0.0),
                       task_id=task_ids.get("uplink")))

    return tasks, uplink_start + uplink_time


def build_mission(segments, velocity, config_time, config_power_robot,
                   localization_interval_distance_m=None,
                   phase_params=None, nap_curve=None, mission_start_lunar_hour=0.0,
                   lunar_day_hours=708.7, locomotion_experiment=None, locomotion_config=None,
                   egress=None, commissioning=None, nap_constant_s=None, task_ids=None,
                   subphase_params=None, localization_sequence=None, standing_enabled=True):
    """
    segments: list of {"distance": meters, "turns": count}, one per drive leg
    between measurement/downlink stops (see path_analyzer.build_drive_segments).

    localization_sequence: ordered list of tasks run every time
    localization_interval_distance_m fires, instead of a single turn -- see
    generate_waypoint_tasks/mission_ops_config.localization_sequence. None
    (default) matches today's single-turn behavior.

    standing_enabled: mission-wide on/off switch for the standing/settle
    task (see generate_waypoint_tasks/mission_ops_config.standing_enabled).
    True (default) matches today's behavior.

    task_ids: optional {key: "OPS-xx" or None} (see
    user_data/config_robot/robot_task_ids_config.py), passed straight
    through to generate_waypoint_tasks. None (default) leaves every task's
    task_id unset -- Egress/Commissioning/Locomotion Test tasks always have
    it unset, since they aren't in the Excel CONOPS at all.

    egress/commissioning: optional {"duration_s": ..., "energy_wh": ...}
    each (see mission_ops_config.py) -- two independent fixed-cost tasks
    inserted right at the start of the timeline (t=0), egress then
    commissioning, before any drive segment. Either can be set without the
    other. mission_start_lunar_hour anchors to t=0 (landing), so together
    they push when the actual traverse/ops begins later in the lunar day by
    however long they take -- nap_curve timing (driven by real elapsed t)
    accounts for that automatically. None (default, for either) skips it.

    localization_interval_distance_m: force a localization sequence roughly
    every this many meters walked -- one running cumulative-distance total
    across the whole route (all segments, all phases/subphases), incremented
    by each segment's distance and reset to 0 whenever it fires, so the
    real-world spacing between localization events stays constant even
    where stop spacing itself varies a lot between phases/subphases (unlike
    a stop-count-based interval, which would fire far more often during a
    tightly-spaced subphase). None (default) never forces one.

    phase_params: optional {phase: {"num_pictures": N, "special_sequence":
    [...], "task_intervals": {...}}} (see mission_ops_config.py).
    num_pictures drives the picture count per ordinary stop in that phase.
    special_sequence (a list of TASK_CATALOG keys, see
    generate_waypoint_tasks) is required for any phase that can have
    segments tagged "special" by build_drive_segments -- it replaces
    STOP_TASK_SEQUENCE entirely at that stop. task_intervals ({key: N} for
    grav_measurement/spec_measurement/scicam_measurement/relay) makes that
    task run only every Nth measurement/stop instead of every stop -- a
    stop-count cadence, unlike localization_interval_distance_m below
    (which is distance-based); keys not listed default to every stop
    (interval 1), and this never applies to special stops. None (default)
    keeps today's behavior: exactly 1
    picture per stop, every task every stop, never special.

    subphase_params: optional {key: {same shape as one phase_params[phase]
    dict}} (see mission_ops_config.py) -- for any segment build_drive_
    segments tagged "subphase": key (a distance window around a user-marked
    waypoint, e.g. "grav_dense_center" -- see build_drive_segments'
    subphase_windows), this dict *completely replaces* phase_params[phase]
    as the source of num_pictures/task_intervals/etc. for that stop, not
    just one field -- build_drive_segments has already used this same
    subphase's own min_distance/max_distance to decide the stop spacing
    itself, so "every Nth stop" here means every Nth stop *of this
    subphase*. Its task_intervals counter resets to 0 the moment a run of
    segments tagged with this key begins, independent of the parent
    phase's own cadence (which keeps advancing underneath, resuming
    unaffected once the subphase ends). A key with no entry in
    subphase_params falls back to an empty dict (every task every stop,
    1 picture) rather than erroring.

    nap_curve/mission_start_lunar_hour/lunar_day_hours/nap_constant_s: see
    generate_waypoint_tasks/nap_duration_s. nap_curve=None and
    nap_constant_s=None (both defaults) keeps every nap at the flat
    config_time["nap_time"]; nap_constant_s overrides that with a fixed
    value of your own instead (nap_curve takes priority if both are set).

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

    for name, block in (("Egress", egress), ("Commissioning", commissioning)):
        if block:
            duration = block["duration_s"]
            power_w = block["energy_wh"] * 3600 / duration
            timeline.append(Task(name, "rover", t, t + duration, phase=0, power_w=power_w))
            t += duration

    measurement_count = 0
    special_occurrence_count = 0
    subphase_measurement_count = 0
    previous_subphase_key = None
    # One running cumulative-distance total across the whole route (all
    # phases/subphases, never reset by subphase entry/exit like
    # subphase_measurement_count is) -- see localization_interval_distance_m.
    distance_since_localization = 0.0
    locomotion_trigger_occurrences = set(
        (locomotion_experiment or {}).get("trigger_after_special_occurrence", []))

    for seg in segments:

        travel_time = seg["distance"] / velocity
        turn_time = seg["turns"] * config_time.get("turn_time", 0)

        measurement_count += 1
        distance_since_localization += seg["distance"]
        localization = (bool(localization_interval_distance_m) and
                         distance_since_localization >= localization_interval_distance_m)
        if localization:
            distance_since_localization = 0.0

        phase = seg.get("phase", 0)
        special = seg.get("special", False)
        subphase_key = seg.get("subphase")

        # A subphase completely swaps the parameter source (not just one
        # field) and counts its own "every Nth stop" from window entry,
        # independent of the parent phase's measurement_count (which keeps
        # advancing underneath and resumes unaffected once the subphase
        # ends).
        if subphase_key is not None:
            if subphase_key != previous_subphase_key:
                subphase_measurement_count = 0
            subphase_measurement_count += 1
            params = (subphase_params or {}).get(subphase_key, {})
            local_measurement_count = subphase_measurement_count
        else:
            params = (phase_params or {}).get(phase, {})
            local_measurement_count = measurement_count
        previous_subphase_key = subphase_key

        repeat_counts = {"picture": params.get("num_pictures", 1)}
        special_sequence = params.get("special_sequence") if special else None

        task_intervals = params.get("task_intervals", {})
        skip_tasks = {key for key, interval in task_intervals.items()
                      if interval and local_measurement_count % interval != 0}

        # spec_warmup always mirrors spec_measurement's own skip decision --
        # there's no separate task_intervals entry for it, since warming up
        # the instrument only makes sense directly before an actual
        # measurement (see STOP_TASK_SEQUENCE)
        if "spec_measurement" in skip_tasks:
            skip_tasks = skip_tasks | {"spec_warmup"}
        else:
            skip_tasks = skip_tasks - {"spec_warmup"}

        tasks, t = generate_waypoint_tasks(
            t, travel_time, turn_time, config_time, config_power_robot,
            phase=phase, localization=localization, special=special,
            repeat_counts=repeat_counts, special_sequence=special_sequence,
            nap_curve=nap_curve, mission_start_lunar_hour=mission_start_lunar_hour,
            lunar_day_hours=lunar_day_hours, skip_tasks=skip_tasks, nap_constant_s=nap_constant_s,
            task_ids=task_ids, localization_sequence=localization_sequence,
            standing_enabled=standing_enabled
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

def plot_timeline(tasks, show=True, velocity=None, phase_names=None, time_offset_h=0.0):
    """
    velocity: rover drive speed [m/s]. If given, each "drive" bar is
    annotated with the distance covered in that leg (distance = velocity *
    duration) and the axes title shows the cumulative distance driven.

    phase_names: names for task.phase 0, 1, ... (see marius_hills_mission.
    run_mission_planning). If given, a faint dashed line marks each phase
    change, labeled with the phase it's entering.

    time_offset_h: shifts every plotted position (not durations/widths) by
    this many hours -- e.g. mission_config.mission_start_lunar_hour, so the
    axis reads as absolute lunar-day time instead of always starting at 0.
    Purely a display shift; doesn't change any task's actual start/end.
    """
    def to_min(t_seconds):
        return t_seconds / 60 + time_offset_h * 60

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
        start_time = to_min(task.start)
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
    ax.set_xlabel(f"Time [min] (lunar hour {time_offset_h:.1f} onward)" if time_offset_h else "Time [min]")
    if velocity is not None:
        ax.set_title(f"Total distance driven: {cumulative_distance_m:.0f} m")

    # thin, faint separator at the start of every stop (marked by "Walk"),
    # so consecutive stops are easy to tell apart at a glance
    for task in tasks:
        if task.name == "Walk":
            ax.axvline(to_min(task.start), color="#c3c2b7", linewidth=0.6, alpha=0.4, zorder=0)

    # phase-change markers: a faint dashed line at the midpoint between the
    # last task of one phase and the first task of the next (tasks aren't
    # strictly ordered by phase at the boundary -- earth-side tasks of the
    # last phase-0 stop can end after the first phase-1 stop's rover tasks
    # start -- so the midpoint keeps the line out of either stop's bars)
    phases_present = sorted(set(task.phase for task in tasks))
    for p0, p1 in zip(phases_present[:-1], phases_present[1:]):
        p0_end = to_min(max(task.end for task in tasks if task.phase == p0))
        p1_start = to_min(min(task.start for task in tasks if task.phase == p1))
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


def plot_timeline_interactive(tasks, velocity=None, phase_names=None, time_offset_h=0.0):
    """
    Interactive Plotly equivalent of plot_timeline, saved as a self-contained
    .html file (see marius_hills_mission.run_mission_planning). Opens in a
    browser with the same scroll-to-zoom/drag-to-pan behavior as the live
    matplotlib window -- unlike a static PNG/PDF, the axis re-renders at
    whatever zoom level you're at, so it never runs out of legible ticks.

    time_offset_h: see plot_timeline -- shifts every plotted position (not
    durations) by this many hours, e.g. mission_config.mission_start_lunar_hour.
    """
    def to_min(t_seconds):
        return t_seconds / 60 + time_offset_h * 60

    present = dict.fromkeys(task.name for task in tasks)
    names = [n for n in TASK_ROW_ORDER if n in present]
    names += [n for n in present if n not in names]

    fig = go.Figure()

    cumulative_distance_m = 0.0
    for actor, color in ACTOR_COLORS.items():
        actor_tasks = [t for t in tasks if t.actor == actor]
        if not actor_tasks:
            continue

        starts_min = [to_min(t.start) for t in actor_tasks]
        durations_min = [(t.end - t.start) / 60 for t in actor_tasks]
        hover = [f"{t.name}<br>start: {to_min(t.start):.1f} min<br>"
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
                    x=to_min(t.start + (t.end - t.start) / 2), y=t.name,
                    text=f"{leg_distance_m:.0f} m", showarrow=False, font=dict(size=14))

    # thin, faint separator at the start of every stop (marked by "Walk"),
    # so consecutive stops are easy to tell apart at a glance
    for t in tasks:
        if t.name == "Walk":
            fig.add_vline(x=to_min(t.start), line_color="#c3c2b7", line_width=0.6,
                           opacity=0.4, layer="below")

    phases_present = sorted(set(task.phase for task in tasks))
    for p0, p1 in zip(phases_present[:-1], phases_present[1:]):
        p0_end = to_min(max(task.end for task in tasks if task.phase == p0))
        p1_start = to_min(min(task.start for task in tasks if task.phase == p1))
        boundary = (p0_end + p1_start) / 2
        fig.add_vline(x=boundary, line_dash="dash", line_color="#898781", opacity=0.6)
        if phase_names and p1 < len(phase_names):
            fig.add_annotation(x=boundary, y=1.0, yref="paper", yanchor="bottom",
                                text=phase_names[p1], showarrow=False,
                                font=dict(size=17, color="#52514e"), xanchor="left")

    # Hour ticks alongside the minute value on every label -- stepped by a
    # "nice" number of hours (from the candidates below) so a long mission
    # doesn't end up with hundreds of ticks; computed once across the whole
    # plotted range rather than tracking interactive zoom/pan.
    all_min = [to_min(t.start) for t in tasks] + [to_min(t.end) for t in tasks]
    min_min, max_min = min(all_min), max(all_min)
    total_hours = (max_min - min_min) / 60
    hour_candidates = (1, 2, 3, 6, 12, 24, 48, 72, 96, 168)
    hour_step = next((s for s in hour_candidates if total_hours / s <= 20),
                      168 * math.ceil(total_hours / 168 / 20))
    tick_start = math.floor(min_min / 60 / hour_step) * hour_step * 60
    tickvals = []
    v = tick_start
    while v <= max_min:
        tickvals.append(v)
        v += hour_step * 60
    ticktext = [f"{tv:.0f} min<br>{tv / 60:.1f} h" for tv in tickvals]

    # Header shows what plot_timeline used to put on the x-axis title
    # (starting lunar hour) instead, alongside the distance summary.
    title_lines = []
    if velocity is not None:
        title_lines.append(f"Total distance driven: {cumulative_distance_m:.0f} m")
    if time_offset_h:
        title_lines.append(f"Start: lunar hour {time_offset_h:.1f}")

    fig.update_layout(
        barmode="overlay",
        yaxis=dict(categoryorder="array", categoryarray=names),
        xaxis=dict(title="Time [min / h]", tickmode="array", tickvals=tickvals, ticktext=ticktext),
        title="<br>".join(title_lines) if title_lines else None,
        legend_title_text="Actor",
        height=600, width=1400,
        margin=dict(t=100),
    )

    return fig



