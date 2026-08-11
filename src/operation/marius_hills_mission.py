import os
from collections import defaultdict

import matplotlib.pyplot as plt

from src.operation import conops_planner
from src.operation import path_analyzer


def generate_mission_report(timeline, segments):
    durations = defaultdict(float)
    for task in timeline:
        durations[task.name] += task.end - task.start

    mission_time = max((t.end for t in timeline), default=0.0)
    walking_distance = sum(seg["distance"] for seg in segments)

    def report_line(name, value):
        print(f"{name:25s}: {value}")

    print("\n============== Mission Summary ==============\n")
    report_line("Total mission time", f"{mission_time / 3600:.2f} h")
    report_line("Walking distance", f"{walking_distance:.2f} m")
    report_line("Number of stops", f"{len(segments)}\n")

    print("Activity breakdown")
    print("---------------------------------------------")
    for name, total in durations.items():
        report_line(name, f"{total / 3600:.2f} h")
    print("\n=============================================\n")

    return {
        "mission_time_h": mission_time / 3600,
        "walking_distance_m": walking_distance,
        "num_stops": len(segments),
        "activity_breakdown_h": {name: total / 3600 for name, total in durations.items()},
    }


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

    segments = path_analyzer.build_drive_segments(path, planning_resolution, stop_distance_m, **turn_kwargs)

    velocity = mission_config["robot_velocity"]
    timeline = conops_planner.build_mission(segments, velocity, robot_time_config, robot_power_config)

    fig = conops_planner.plot_timeline(timeline, show=False, velocity=velocity)
    fig.savefig(timeline_image_path, dpi=300)
    plt.close(fig)

    report = generate_mission_report(timeline, segments)

    # serialized for provenance/playback -- lets a later viewer (see
    # src/operation/mission_playback.py) reconstruct the timeline exactly as
    # planned, without needing to re-run build_mission
    timeline_serialized = [
        {"name": t.name, "actor": t.actor, "start": t.start, "end": t.end} for t in timeline]

    return {"mission_config": mission_config, "segments": segments, "report": report,
            "timeline": timeline_serialized}
