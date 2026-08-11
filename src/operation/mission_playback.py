"""
Experimental: an interactive viewer linking the mission timeline to the
rover's position on the map. Hover over the timeline (bottom panel) to see
where the rover is at that moment, highlighted on the map (top panel), along
with elapsed time and distance driven so far.
"""
from typing import List, Optional, Tuple, Union

import matplotlib.pyplot as plt
import numpy as np

from src.operation.waypoint_picker import (
    _draw_boulder_safety_halos, _draw_map_meters, _draw_path_corridor,
    _draw_start_circle, _draw_zones, _grid_to_m)

TaskLike = Union[dict, object]


def _task_field(task: TaskLike, key: str):
    return task[key] if isinstance(task, dict) else getattr(task, key)


def cumulative_path_distance_m(path: List[Tuple[float, float]], resolution: float) -> np.ndarray:
    """Cumulative arc length in meters at each path vertex, path[0] -> 0."""
    pts = np.array(path, dtype=float)
    seg_lengths_m = np.linalg.norm(np.diff(pts, axis=0), axis=1) / resolution
    return np.concatenate([[0.0], np.cumsum(seg_lengths_m)])


def position_at_distance(path: List[Tuple[float, float]], cum_dist_m: np.ndarray,
                          distance_m: float) -> Tuple[float, float]:
    """Interpolated (row, col) at a given cumulative distance along the path."""
    if distance_m <= 0:
        return path[0]
    if distance_m >= cum_dist_m[-1]:
        return path[-1]

    i = int(np.searchsorted(cum_dist_m, distance_m))
    d0, d1 = cum_dist_m[i - 1], cum_dist_m[i]
    frac = 0.0 if d1 == d0 else (distance_m - d0) / (d1 - d0)
    r0, c0 = path[i - 1]
    r1, c1 = path[i]
    return r0 + (r1 - r0) * frac, c0 + (c1 - c0) * frac


def distance_at_time(timeline: List[TaskLike], velocity: float, t_s: float) -> float:
    """Cumulative drive distance (m) reached by mission-clock time t_s (s)."""
    drive_tasks = sorted(
        (task for task in timeline if _task_field(task, "name") == "drive"),
        key=lambda task: _task_field(task, "start"))

    cumulative_m = 0.0
    for task in drive_tasks:
        start, end = _task_field(task, "start"), _task_field(task, "end")
        if t_s < start:
            return cumulative_m
        if t_s <= end:
            return cumulative_m + velocity * (t_s - start)
        cumulative_m += velocity * (end - start)

    return cumulative_m


def show_mission_playback(grid: np.ndarray, path: List[Tuple[int, int]], resolution: float,
                           timeline: List[TaskLike], velocity: float,
                           waypoints: Optional[List[Tuple[int, int]]] = None,
                           start: Optional[Tuple[int, int]] = None,
                           landing_zone_radius_m: Optional[float] = None,
                           start_area_radius_m: Optional[float] = None,
                           start_circle_center_m: Optional[Tuple[float, float]] = None,
                           start_circle_radius_m: Optional[float] = None,
                           boulder_centers_m: Optional[np.ndarray] = None,
                           boulder_radii_m: Optional[np.ndarray] = None,
                           boulder_safety_margin_m: Optional[float] = None,
                           corridor_width_m: Optional[float] = None,
                           crop_center_m: Optional[Tuple[float, float]] = None,
                           crop_size_m: Optional[float] = None):
    """
    Experimental. Opens a two-panel window: the map with the planned route
    on top, the task timeline (Gantt-style, as in conops_planner.plot_timeline)
    on the bottom. Move the mouse over the timeline to see a magenta marker
    on the map jump to the rover's position at that moment, with elapsed
    time / distance driven shown in the map's corner. Close the window when
    done. If crop_center_m/crop_size_m are given, the map panel opens zoomed
    to a crop_size_m x crop_size_m square around crop_center_m instead of the
    full map -- it's still a live matplotlib axes, so the toolbar can zoom/pan
    it further or back out to the full map at any time.
    """
    cum_dist_m = cumulative_path_distance_m(path, resolution)

    fig, (ax_map, ax_time) = plt.subplots(2, 1, figsize=(10, 13), height_ratios=[2, 1])

    _draw_map_meters(ax_map, grid, resolution)
    _draw_start_circle(ax_map, start_circle_center_m, start_circle_radius_m)
    _draw_boulder_safety_halos(ax_map, boulder_centers_m, boulder_radii_m, boulder_safety_margin_m)
    _draw_path_corridor(ax_map, path, resolution, corridor_width_m)
    _draw_zones(ax_map, start, resolution, landing_zone_radius_m, start_area_radius_m)

    if path:
        path_m = [_grid_to_m(r, c, resolution) for r, c in path]
        ax_map.plot([p[0] for p in path_m], [p[1] for p in path_m], "b-", linewidth=2, alpha=0.6, label="Path")
    if waypoints:
        for i, (r, c) in enumerate(waypoints, start=1):
            x_m, y_m = _grid_to_m(r, c, resolution)
            ax_map.plot(x_m, y_m, "o", color="yellow", markeredgecolor="black", markersize=8, zorder=5)
            ax_map.annotate(str(i), (x_m, y_m), textcoords="offset points", xytext=(5, 5),
                             color="yellow", fontsize=10, weight="bold", zorder=6)

    cursor_marker, = ax_map.plot([], [], "m*", markersize=22, zorder=10)
    info_text = ax_map.text(0.02, 0.98, "", transform=ax_map.transAxes, va="top", ha="left",
                             color="magenta", fontsize=11, weight="bold")
    ax_map.legend(loc="upper right")

    if crop_center_m is not None and crop_size_m is not None:
        half = crop_size_m / 2
        ax_map.set_xlim(crop_center_m[0] - half, crop_center_m[0] + half)
        ax_map.set_ylim(crop_center_m[1] - half, crop_center_m[1] + half)

    names = list(dict.fromkeys(_task_field(t, "name") for t in timeline))
    y_map = {name: i for i, name in enumerate(names)}
    for task in timeline:
        start_min = _task_field(task, "start") / 60
        duration_min = (_task_field(task, "end") - _task_field(task, "start")) / 60
        ax_time.broken_barh([(start_min, duration_min)], (y_map[_task_field(task, "name")] - 0.4, 0.5))
    ax_time.set_yticks(list(y_map.values()))
    ax_time.set_yticklabels(list(y_map.keys()))
    ax_time.set_xlabel("Time [min]")
    ax_time.set_title("Hover here to scrub through the mission")

    cursor_line = ax_time.axvline(0, color="magenta", linewidth=1, visible=False)

    def on_move(event):
        if event.inaxes != ax_time or event.xdata is None:
            return

        t_s = event.xdata * 60
        cursor_line.set_xdata([event.xdata, event.xdata])
        cursor_line.set_visible(True)

        distance_m = distance_at_time(timeline, velocity, t_s)
        row, col = position_at_distance(path, cum_dist_m, distance_m)
        x_m, y_m = _grid_to_m(row, col, resolution)

        cursor_marker.set_data([x_m], [y_m])
        info_text.set_text(f"t = {t_s / 60:.1f} min\ndistance driven = {distance_m:.0f} m")

        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("motion_notify_event", on_move)

    plt.tight_layout()
    plt.show()
