import os
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

from src.operation import conops_planner
from src.operation import path_analyzer


def _phase_metrics(tasks, segments):
    durations = defaultdict(float)
    energy_wh = defaultdict(float)
    counts = defaultdict(int)
    for task in tasks:
        duration_s = task.end - task.start
        durations[task.name] += duration_s
        energy_wh[task.name] += task.power_w * duration_s / 3600
        counts[task.name] += 1

    # wall-clock span, not sum(durations) -- rover and earth tasks run in
    # parallel, so summing would double-count that overlap
    starts = [t.start for t in tasks]
    ends = [t.end for t in tasks]
    mission_time = (max(ends) - min(starts)) if tasks else 0.0

    return {
        "mission_time_h": mission_time / 3600,
        "walking_distance_m": sum(seg["distance"] for seg in segments),
        "num_stops": len(segments),
        "special_stop_count": sum(1 for seg in segments if seg.get("special")),
        "activity_breakdown_h": {name: total / 3600 for name, total in durations.items()},
        "activity_energy_wh": dict(energy_wh),
        "total_energy_wh": sum(energy_wh.values()),
        "activity_count": dict(counts),
    }


def _report_to_excel(report, excel_path, special_sequence_name):
    """
    Same numbers as the printed/text report, reshaped into two tidy tables
    (one row per scope/activity, easy to filter/pivot/chart in Excel)
    instead of the indented plain-text layout.
    """
    def overview_row(scope, r):
        return {
            "Scope": scope,
            "Mission time (h)": r["mission_time_h"],
            "Walking distance (m)": r["walking_distance_m"],
            "Number of stops": r["num_stops"],
            f"{special_sequence_name} (x)": r["special_stop_count"],
            "Total energy (Wh)": r["total_energy_wh"],
        }

    def activity_rows(scope, r):
        return [{
            "Scope": scope,
            "Activity": name,
            "Count": r["activity_count"][name],
            "Time (h)": r["activity_breakdown_h"][name],
            "Energy (Wh)": r["activity_energy_wh"][name],
        } for name in r["activity_breakdown_h"]]

    phases = report.get("phases", {})
    overview_df = pd.DataFrame(
        [overview_row("Overall", report)] + [overview_row(name, r) for name, r in phases.items()])
    activity_df = pd.DataFrame(
        activity_rows("Overall", report) + [row for name, r in phases.items() for row in activity_rows(name, r)])

    with pd.ExcelWriter(excel_path) as writer:
        overview_df.to_excel(writer, sheet_name="Overview", index=False)
        activity_df.to_excel(writer, sheet_name="Activity Breakdown", index=False)


def generate_mission_report(timeline, segments, phase_names=None, excel_path=None,
                             special_sequence_name=None):
    """
    Same summary as before, but built up as text first: always printed to
    stdout, and also written to excel_path (if given, see _report_to_excel)
    so it survives past the terminal in a form you can filter/pivot -- e.g.
    alongside Mission_Timeline.png/Route_Plot.html in the run folder, see
    run_mission_planning.

    special_sequence_name: display label for special stops (see
    mission_ops_config.phase_params[1].special_sequence_name), shown as its
    own "Nx" line wherever special stops occurred, separate from the
    individual task rows they're made of. None (default) falls back to a
    generic "Special stops" label.
    """
    special_sequence_name = special_sequence_name or "Special stops"
    report = _phase_metrics(timeline, segments)
    lines = []

    def line(text=""):
        lines.append(text)

    def report_line(name, value, indent=""):
        lines.append(f"{indent}{name:25s}: {value}")

    line("\n============== Mission Summary ==============\n")
    report_line("Total mission time", f"{report['mission_time_h']:.2f} h")
    report_line("Walking distance", f"{report['walking_distance_m']:.2f} m")
    report_line("Number of stops", f"{report['num_stops']}")
    if report["special_stop_count"]:
        report_line(special_sequence_name, f"{report['special_stop_count']}x")
    report_line("Total energy", f"{report['total_energy_wh']:.1f} Wh\n")

    line("Activity breakdown (count / time / energy)")
    line("---------------------------------------------")
    for name, total_h in report["activity_breakdown_h"].items():
        report_line(name, f"{report['activity_count'][name]}x  /  "
                           f"{total_h:.2f} h  /  {report['activity_energy_wh'][name]:.1f} Wh")

    if phase_names:
        phases = {}
        for phase_idx, phase_name in enumerate(phase_names):
            phase_tasks = [t for t in timeline if t.phase == phase_idx]
            phase_segments = [s for s in segments if s["phase"] == phase_idx]
            phases[phase_name] = _phase_metrics(phase_tasks, phase_segments)

        line("\nBy phase")
        line("---------------------------------------------")
        for phase_name, phase_report in phases.items():
            line(f"\n{phase_name}")
            report_line("Mission time", f"{phase_report['mission_time_h']:.2f} h", indent="  ")
            report_line("Walking distance", f"{phase_report['walking_distance_m']:.2f} m", indent="  ")
            report_line("Number of stops", f"{phase_report['num_stops']}", indent="  ")
            if phase_report["special_stop_count"]:
                report_line(special_sequence_name, f"{phase_report['special_stop_count']}x", indent="  ")
            report_line("Total energy", f"{phase_report['total_energy_wh']:.1f} Wh", indent="  ")
            for name, total_h in phase_report["activity_breakdown_h"].items():
                report_line(name, f"{phase_report['activity_count'][name]}x  /  "
                                   f"{total_h:.2f} h  /  {phase_report['activity_energy_wh'][name]:.1f} Wh",
                            indent="    ")

        report["phases"] = phases

    line("\n=============================================\n")

    text = "\n".join(lines)
    print(text)
    if excel_path is not None:
        _report_to_excel(report, excel_path, special_sequence_name)

    return report


def run_mission_planning(experiment, mission_config, robot_time_config, robot_power_config,
                          timeline_image_path: str | os.PathLike = "Marius_Hills_Mission_Timeline.png",
                          locomotion_config=None):
    """
    Build a mission timeline from an already-computed path (as saved by
    marius_hills_review.run_simulation) instead of re-running path planning.

    locomotion_config: time/power values for mission_config["locomotion_
    experiment"]'s tasks (see user_data/config_robot/
    locomotion_experiment_config.py). None (default) means no locomotion-
    experiment blocks get inserted, even if locomotion_experiment is set.
    """
    if not experiment["result"]["success"] or experiment["path"] is None:
        raise ValueError("experiment has no successful path to plan a mission from")

    path = experiment["path"]
    planning_resolution = experiment["planning_resolution"]

    # Per-phase stop spacing (see mission_ops_config.phase_params) -- falls
    # back to flat min_distance/max_distance keys for callers that don't
    # set phase_params (e.g. an older saved experiment with no phase split).
    phase_params = mission_config.get("phase_params")
    if phase_params:
        step_m = {p: (v["min_distance"] + v["max_distance"]) / 2 for p, v in phase_params.items()}
    else:
        step_m = (mission_config["min_distance"] + mission_config["max_distance"]) / 2

    # window_m / angle_threshold_deg are optional: if the config omits either,
    # don't pass that kwarg at all so build_drive_segments' own default
    # parameter applies -- avoids duplicating those defaults here.
    turn_kwargs = {}
    if "window_m" in mission_config:
        turn_kwargs["window_m"] = mission_config["window_m"]
    if "angle_threshold_deg" in mission_config:
        turn_kwargs["angle_threshold_deg"] = mission_config["angle_threshold_deg"]

    # Report split into phases (start -> first waypoint, first waypoint ->
    # the rest) when the caller passes the full waypoint list the path was
    # planned through and mission_config names the phases. The boundary is
    # the path-vertex index of the first waypoint -- an exact vertex of
    # `path` since path_planning_multi/simplify_path_line_of_sight both
    # pass through every waypoint unchanged.
    phase_names = mission_config.get("phase_names")
    waypoints = experiment.get("waypoints")
    phase_boundary_idx = None
    if phase_names and waypoints and len(waypoints) > 1:
        phase_boundary_idx = path.index(tuple(waypoints[1]))

    # Special waypoints (a subset of `waypoints` the caller flagged -- see
    # waypoint_picker.pick_special_waypoints) -- path-vertex indices,
    # phase-gated inside build_drive_segments itself (only takes effect if
    # the segment they snap onto is actually phase 1/pit_ops).
    special_waypoints = experiment.get("special_waypoints")
    special_trigger_idxs = [path.index(tuple(wp)) for wp in special_waypoints] if special_waypoints else None

    segments = path_analyzer.build_drive_segments(
        path, planning_resolution, step_m,
        phase_boundary_idx=phase_boundary_idx, special_trigger_idxs=special_trigger_idxs, **turn_kwargs)

    # Force a "localization turn" every Nth measurement (i.e. every Nth
    # stop/segment -- see conops_planner.build_mission).
    localization_interval = mission_config.get("localization_interval_measurements")

    # Nap duration follows a lunar-day thermal/power cycle instead of a
    # flat value when nap_curve is set (see conops_planner.nap_duration_s).
    nap_curve = mission_config.get("nap_curve")
    mission_start_lunar_hour = mission_config.get("mission_start_lunar_hour", 0.0)
    lunar_day_hours = mission_config.get("lunar_day_hours", 708.7)

    # Locomotion-experiment blocks inserted after specific special-waypoint
    # occurrences (see conops_planner.build_mission/generate_locomotion_
    # experiment_tasks).
    locomotion_experiment = mission_config.get("locomotion_experiment")

    # One-time egress/commissioning block at t=0, before any drive segment
    # (see conops_planner.build_mission).
    egress_commissioning = mission_config.get("egress_commissioning")

    velocity = mission_config["robot_velocity"]
    timeline = conops_planner.build_mission(
        segments, velocity, robot_time_config, robot_power_config,
        localization_interval=localization_interval, phase_params=phase_params,
        nap_curve=nap_curve, mission_start_lunar_hour=mission_start_lunar_hour,
        lunar_day_hours=lunar_day_hours, locomotion_experiment=locomotion_experiment,
        locomotion_config=locomotion_config, egress_commissioning=egress_commissioning)

    fig = conops_planner.plot_timeline(timeline, show=False, velocity=velocity, phase_names=phase_names,
                                        time_offset_h=mission_start_lunar_hour)
    fig.savefig(timeline_image_path, dpi=300)
    plt.close(fig)

    # interactive copy alongside the PNG -- opens in a browser with the same
    # scroll-to-zoom/drag-to-pan behavior as the live matplotlib window, so
    # the axis stays legible at any zoom level (a static PNG/PDF can't do
    # that: its ticks are baked in for the full-timeline view)
    interactive_fig = conops_planner.plot_timeline_interactive(
        timeline, velocity=velocity, phase_names=phase_names, time_offset_h=mission_start_lunar_hour)
    interactive_fig.write_html(Path(timeline_image_path).with_suffix(".html"))

    special_sequence_name = (phase_params or {}).get(1, {}).get("special_sequence_name")
    excel_path = Path(timeline_image_path).with_name("Mission_Summary.xlsx")
    report = generate_mission_report(timeline, segments, phase_names=phase_names, excel_path=excel_path,
                                      special_sequence_name=special_sequence_name)

    # serialized for provenance/playback -- lets a later viewer (see
    # src/operation/mission_playback.py) reconstruct the timeline exactly as
    # planned, without needing to re-run build_mission
    timeline_serialized = [
        {"name": t.name, "actor": t.actor, "start": t.start, "end": t.end, "phase": t.phase}
        for t in timeline]

    return {"mission_config": mission_config, "segments": segments, "report": report,
            "timeline": timeline_serialized}
