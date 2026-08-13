import os
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt

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
        "activity_breakdown_h": {name: total / 3600 for name, total in durations.items()},
        "activity_energy_wh": dict(energy_wh),
        "total_energy_wh": sum(energy_wh.values()),
        "activity_count": dict(counts),
    }


def generate_mission_report(timeline, segments, phase_names=None):
    report = _phase_metrics(timeline, segments)

    def report_line(name, value, indent=""):
        print(f"{indent}{name:25s}: {value}")

    print("\n============== Mission Summary ==============\n")
    report_line("Total mission time", f"{report['mission_time_h']:.2f} h")
    report_line("Walking distance", f"{report['walking_distance_m']:.2f} m")
    report_line("Number of stops", f"{report['num_stops']}")
    report_line("Total energy", f"{report['total_energy_wh']:.1f} Wh\n")

    print("Activity breakdown (count / time / energy)")
    print("---------------------------------------------")
    for name, total_h in report["activity_breakdown_h"].items():
        report_line(name, f"{report['activity_count'][name]}x  /  "
                           f"{total_h:.2f} h  /  {report['activity_energy_wh'][name]:.1f} Wh")

    if phase_names:
        phases = {}
        for phase_idx, phase_name in enumerate(phase_names):
            phase_tasks = [t for t in timeline if t.phase == phase_idx]
            phase_segments = [s for s in segments if s["phase"] == phase_idx]
            phases[phase_name] = _phase_metrics(phase_tasks, phase_segments)

        print("\nBy phase")
        print("---------------------------------------------")
        for phase_name, phase_report in phases.items():
            print(f"\n{phase_name}")
            report_line("Mission time", f"{phase_report['mission_time_h']:.2f} h", indent="  ")
            report_line("Walking distance", f"{phase_report['walking_distance_m']:.2f} m", indent="  ")
            report_line("Number of stops", f"{phase_report['num_stops']}", indent="  ")
            report_line("Total energy", f"{phase_report['total_energy_wh']:.1f} Wh", indent="  ")
            for name, total_h in phase_report["activity_breakdown_h"].items():
                report_line(name, f"{phase_report['activity_count'][name]}x  /  "
                                   f"{total_h:.2f} h  /  {phase_report['activity_energy_wh'][name]:.1f} Wh",
                            indent="    ")

        report["phases"] = phases

    print("\n=============================================\n")
    return report


def run_mission_planning(experiment, mission_config, robot_time_config, robot_power_config,
                          timeline_image_path: str | os.PathLike = "Marius_Hills_Mission_Timeline.png"):
    """
    Build a mission timeline from an already-computed path (as saved by
    marius_hills_review.run_simulation) instead of re-running path planning.
    """
    if not experiment["result"]["success"] or experiment["path"] is None:
        raise ValueError("experiment has no successful path to plan a mission from")

    path = experiment["path"]
    planning_resolution = experiment["planning_resolution"]

    stop_distance_m = (mission_config["min_distance"] + mission_config["max_distance"]) / 2

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

    segments = path_analyzer.build_drive_segments(
        path, planning_resolution, stop_distance_m,
        phase_boundary_idx=phase_boundary_idx, **turn_kwargs)

    # Force a "localization turn" every Nth measurement (i.e. every Nth
    # stop/segment -- see conops_planner.build_mission).
    localization_interval = mission_config.get("localization_interval_measurements")

    velocity = mission_config["robot_velocity"]
    timeline = conops_planner.build_mission(
        segments, velocity, robot_time_config, robot_power_config,
        localization_interval=localization_interval)

    fig = conops_planner.plot_timeline(timeline, show=False, velocity=velocity, phase_names=phase_names)
    fig.savefig(timeline_image_path, dpi=300)
    plt.close(fig)

    # interactive copy alongside the PNG -- opens in a browser with the same
    # scroll-to-zoom/drag-to-pan behavior as the live matplotlib window, so
    # the axis stays legible at any zoom level (a static PNG/PDF can't do
    # that: its ticks are baked in for the full-timeline view)
    interactive_fig = conops_planner.plot_timeline_interactive(
        timeline, velocity=velocity, phase_names=phase_names)
    interactive_fig.write_html(Path(timeline_image_path).with_suffix(".html"))

    report = generate_mission_report(timeline, segments, phase_names=phase_names)

    # serialized for provenance/playback -- lets a later viewer (see
    # src/operation/mission_playback.py) reconstruct the timeline exactly as
    # planned, without needing to re-run build_mission
    timeline_serialized = [
        {"name": t.name, "actor": t.actor, "start": t.start, "end": t.end, "phase": t.phase}
        for t in timeline]

    return {"mission_config": mission_config, "segments": segments, "report": report,
            "timeline": timeline_serialized}
