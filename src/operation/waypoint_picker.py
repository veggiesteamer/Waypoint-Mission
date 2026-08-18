import base64
import io
import json
import math
from pathlib import Path
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.collections import PatchCollection
import numpy as np
import plotly.graph_objects as go


def _grid_to_m(row: float, col: float, resolution: float) -> Tuple[float, float]:
    return col / resolution, row / resolution


def start_point_on_circle_m(center_m: Tuple[float, float], radius_m: float,
                             angle_deg: float) -> Tuple[float, float]:
    """
    Point at a given bearing on a circle of radius_m around center_m (both
    in meters), in meters. angle_deg is measured counterclockwise from +X
    (standard math convention: 0 deg = east of center, 90 deg = north).
    """
    angle_rad = math.radians(angle_deg)
    return (center_m[0] + radius_m * math.cos(angle_rad),
            center_m[1] + radius_m * math.sin(angle_rad))


def start_point_from_circle(center_m: Tuple[float, float], radius_m: float, angle_deg: float,
                             resolution: float) -> Tuple[int, int]:
    """
    Compute a start point at a given bearing on a circle of radius_m around
    center_m (both in meters), returned in grid pixel coordinates. angle_deg
    is measured counterclockwise from +X (standard math convention: 0 deg =
    east of center, 90 deg = north of center).
    """
    x_m, y_m = start_point_on_circle_m(center_m, radius_m, angle_deg)
    return int(round(y_m * resolution)), int(round(x_m * resolution))


def save_waypoints(path, start: Optional[Tuple[int, int]], waypoints: List[Tuple[int, int]]):
    Path(path).write_text(json.dumps({"start": start, "waypoints": waypoints}))


def load_waypoints(path) -> Tuple[Optional[Tuple[int, int]], List[Tuple[int, int]]]:
    """
    Returns (start, waypoints). start is None if the saved file predates
    interactive start-point picking (a plain waypoint list) or never had one.
    """
    data = json.loads(Path(path).read_text())
    if isinstance(data, list):  # legacy format: plain list of waypoints, no start
        return None, [tuple(wp) for wp in data]
    start = tuple(data["start"]) if data.get("start") is not None else None
    return start, [tuple(wp) for wp in data["waypoints"]]


def save_special_waypoints(path, special_waypoints: List[Tuple[int, int]]):
    Path(path).write_text(json.dumps(special_waypoints))


def load_special_waypoints(path) -> List[Tuple[int, int]]:
    return [tuple(wp) for wp in json.loads(Path(path).read_text())]


def _draw_map_meters(ax, grid: np.ndarray, resolution: float):
    height_m = grid.shape[0] / resolution
    width_m = grid.shape[1] / resolution
    ax.imshow(grid, cmap="binary", origin="lower", extent=[0, width_m, 0, height_m])
    ax.set_xlim(0, width_m)
    ax.set_ylim(0, height_m)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")


def _draw_zones(ax, start: Optional[Tuple[int, int]], resolution: float,
                 landing_zone_radius_m: Optional[float], start_area_radius_m: Optional[float]):
    if start is None:
        return
    x_m, y_m = _grid_to_m(start[0], start[1], resolution)

    if landing_zone_radius_m:
        ax.add_patch(Circle((x_m, y_m), landing_zone_radius_m,
                             facecolor="orange", edgecolor="orange", alpha=0.15, label="Landing zone"))
    if start_area_radius_m:
        ax.add_patch(Circle((x_m, y_m), start_area_radius_m,
                             facecolor="red", edgecolor="red", alpha=0.25, label="Start area"))

    ax.plot(x_m, y_m, "go", markersize=12, label="Start")


def _draw_start_circle(ax, start_circle_center_m: Optional[Tuple[float, float]],
                        start_circle_radius_m: Optional[float]):
    if start_circle_center_m is None or not start_circle_radius_m:
        return
    ax.add_patch(Circle(start_circle_center_m, start_circle_radius_m, facecolor="none",
                         edgecolor="cyan", linestyle="--", linewidth=1.5, label="Start circle"))


def _draw_boulder_safety_halos(ax, boulder_centers_m: Optional[np.ndarray],
                                boulder_radii_m: Optional[np.ndarray],
                                safety_margin_m: Optional[float]):
    """
    Faint gray halo around each boulder representing a safety-clearance
    margin. Visual context only, drawn behind the path/markers (low
    zorder) -- it doesn't affect planning.
    """
    if boulder_centers_m is None or len(boulder_centers_m) == 0 or not safety_margin_m:
        return
    patches = [Circle((x, y), r + safety_margin_m) for (x, y), r in zip(boulder_centers_m, boulder_radii_m)]
    collection = PatchCollection(patches, facecolor="gray", edgecolor="none", alpha=0.25, zorder=1)
    ax.add_collection(collection)


def _draw_path_corridor(ax, path: List[Tuple[int, int]], resolution: float,
                         corridor_width_m: Optional[float]):
    """
    Faint band of corridor_width_m along the path, representing the rover's
    own footprint/safety margin as it drives -- built from overlapping
    circles densely sampled along the path so the band has no gaps.
    """
    if not path or not corridor_width_m or len(path) < 2:
        return
    path_m = np.array([_grid_to_m(r, c, resolution) for r, c in path])
    step = corridor_width_m / 2

    dense_points = [path_m[0]]
    for p0, p1 in zip(path_m[:-1], path_m[1:]):
        seg_len = np.linalg.norm(p1 - p0)
        n_steps = max(1, int(np.ceil(seg_len / step)))
        for i in range(1, n_steps + 1):
            dense_points.append(p0 + (p1 - p0) * (i / n_steps))

    patches = [Circle((x, y), corridor_width_m / 2) for x, y in dense_points]
    collection = PatchCollection(patches, facecolor="dimgray", edgecolor="none", alpha=0.15, zorder=1)
    ax.add_collection(collection)


def pick_waypoints(grid: np.ndarray, resolution: float, start: Optional[Tuple[int, int]] = None,
                    landing_zone_radius_m: Optional[float] = None,
                    start_area_radius_m: Optional[float] = None,
                    start_circle_center_m: Optional[Tuple[float, float]] = None,
                    start_circle_radius_m: Optional[float] = None,
                    preview_point_m: Optional[Tuple[float, float]] = None,
                    boulder_centers_m: Optional[np.ndarray] = None,
                    boulder_radii_m: Optional[np.ndarray] = None,
                    boulder_safety_margin_m: Optional[float] = None,
                    start_title: str = "Click to place the landing / start point",
                    waypoint_title: str = "Click to place waypoints (in order) - press 'z' to undo - close window when done"
                    ) -> Tuple[Optional[Tuple[int, int]], List[Tuple[int, int]]]:
    """
    Show the occupancy grid (axes in meters) and let the user click to place
    the mission's start/landing point followed by an ordered sequence of
    waypoints, all returned in grid pixel coordinates. If start is already
    given, it's fixed (drawn immediately) and every click places a waypoint;
    if start is None, the landing zone / start area circles are previewed
    around preview_point_m (falls back to start_circle_center_m if not
    given -- pass the actual point on the start circle, e.g.
    start_point_on_circle_m, rather than its center, so the preview lands
    on a real candidate spot instead of the circle's abstract middle) so
    there's a target to click near, then the first click sets the actual
    start point -- moving those circles to be centered on it instead -- and
    every click after that places a waypoint. Use the normal matplotlib
    toolbar to zoom/pan freely -- a placement only registers if the mouse is
    released within a few pixels of where it was pressed, so a zoom/pan drag
    never drops a stray point, and (unlike checking the toolbar's mode) this
    also keeps working right after the zoom/pan tool was used, without
    needing to click it off first.
    A line grows between placed points so
    the route being built stays visible. Press 'z' to undo the last point
    (a waypoint if any are placed, otherwise the start point itself, if it
    was picked interactively -- restoring the landing zone preview). Returns
    (start, waypoints) once the figure window is closed.
    """
    fig, ax = plt.subplots(figsize=(10, 10))
    _draw_map_meters(ax, grid, resolution)
    _draw_start_circle(ax, start_circle_center_m, start_circle_radius_m)
    _draw_boulder_safety_halos(ax, boulder_centers_m, boulder_radii_m, boulder_safety_margin_m)

    pick_start_interactively = start is None
    picked = {"start": start}
    waypoints: List[Tuple[int, int]] = []
    markers = []
    start_artists = []
    preview_artists = []
    route_line, = ax.plot([], [], "y-", linewidth=2, alpha=0.8)
    preview_center_m = preview_point_m if preview_point_m is not None else start_circle_center_m

    def current_title():
        if picked["start"] is None:
            return start_title
        return f"{waypoint_title}\n(resolution: {resolution:.2f} px/m) - {len(waypoints)} waypoint(s)"

    def show_landing_zone_preview():
        if preview_center_m is None:
            return
        if landing_zone_radius_m:
            preview_artists.append(ax.add_patch(Circle(
                preview_center_m, landing_zone_radius_m,
                facecolor="orange", edgecolor="orange", alpha=0.15, label="Landing zone")))
        if start_area_radius_m:
            preview_artists.append(ax.add_patch(Circle(
                preview_center_m, start_area_radius_m,
                facecolor="red", edgecolor="red", alpha=0.25, label="Start area")))
        ax.legend(loc="upper right")

    def draw_start(row, col):
        for artist in preview_artists:
            artist.remove()
        preview_artists.clear()

        x_m, y_m = _grid_to_m(row, col, resolution)
        if landing_zone_radius_m:
            start_artists.append(ax.add_patch(Circle(
                (x_m, y_m), landing_zone_radius_m,
                facecolor="orange", edgecolor="orange", alpha=0.15, label="Landing zone")))
        if start_area_radius_m:
            start_artists.append(ax.add_patch(Circle(
                (x_m, y_m), start_area_radius_m,
                facecolor="red", edgecolor="red", alpha=0.25, label="Start area")))
        marker, = ax.plot(x_m, y_m, "go", markersize=12, label="Start")
        start_artists.append(marker)
        ax.legend(loc="upper right")

    if picked["start"] is not None:
        draw_start(*picked["start"])
    elif pick_start_interactively:
        show_landing_zone_preview()

    ax.set_title(current_title())

    def route_points_m():
        pts = ([picked["start"]] if picked["start"] is not None else []) + waypoints
        return [_grid_to_m(r, c, resolution) for r, c in pts]

    def refresh():
        pts = route_points_m()
        route_line.set_data([p[0] for p in pts], [p[1] for p in pts])
        ax.set_title(current_title())
        fig.canvas.draw_idle()

    # A "placement" only counts if the mouse is released within this many
    # (screen) pixels of where it was pressed -- distinguishes a click from a
    # zoom-rect/pan drag without needing to know whether the toolbar's
    # zoom/pan tool happens to be toggled on (it stays toggled on after use,
    # which would otherwise block every click that follows).
    CLICK_TOLERANCE_PX = 4
    press_xy = {"px": None}

    def on_press(event):
        press_xy["px"] = (event.x, event.y) if event.inaxes == ax else None

    def on_release(event):
        start_px = press_xy["px"]
        press_xy["px"] = None
        if start_px is None or event.inaxes != ax or event.xdata is None or event.ydata is None:
            return
        if (event.x - start_px[0]) ** 2 + (event.y - start_px[1]) ** 2 > CLICK_TOLERANCE_PX ** 2:
            return  # mouse moved -- a drag (zoom/pan), not a click

        row, col = int(round(event.ydata * resolution)), int(round(event.xdata * resolution))

        if picked["start"] is None:
            picked["start"] = (row, col)
            draw_start(row, col)
        else:
            waypoints.append((row, col))
            x_m, y_m = event.xdata, event.ydata
            marker, = ax.plot(x_m, y_m, "o", color="yellow", markeredgecolor="black", markersize=10, zorder=5)
            label = ax.annotate(str(len(waypoints)), (x_m, y_m), textcoords="offset points",
                                 xytext=(6, 6), color="yellow", fontsize=17, weight="bold", zorder=6)
            markers.append((marker, label))

        refresh()

    def on_key(event):
        if event.key != "z":
            return
        if waypoints:
            waypoints.pop()
            marker, label = markers.pop()
            marker.remove()
            label.remove()
        elif pick_start_interactively and picked["start"] is not None:
            picked["start"] = None
            for artist in start_artists:
                artist.remove()
            start_artists.clear()
            show_landing_zone_preview()
        else:
            return

        refresh()

    fig.canvas.mpl_connect("button_press_event", on_press)
    fig.canvas.mpl_connect("button_release_event", on_release)
    fig.canvas.mpl_connect("key_press_event", on_key)

    plt.show()  # blocks until the window is closed

    return picked["start"], waypoints


def pick_special_waypoints(grid: np.ndarray, resolution: float, waypoints: List[Tuple[int, int]],
                            start: Optional[Tuple[int, int]] = None,
                            landing_zone_radius_m: Optional[float] = None,
                            start_area_radius_m: Optional[float] = None,
                            start_circle_center_m: Optional[Tuple[float, float]] = None,
                            start_circle_radius_m: Optional[float] = None,
                            boulder_centers_m: Optional[np.ndarray] = None,
                            boulder_radii_m: Optional[np.ndarray] = None,
                            boulder_safety_margin_m: Optional[float] = None,
                            click_radius_m: float = 3.0,
                            title: str = "Click a waypoint to mark it special (toggles) "
                                         "- close window when done"
                            ) -> List[Tuple[int, int]]:
    """
    Show the already-picked route `waypoints` on the map (same context as
    pick_waypoints) and let the user click on/near one to toggle whether
    it's "special" -- a richer, independently-defined activity sequence
    (see conops_planner.SPECIAL_STOP_TASK_SEQUENCE) runs at whichever
    regular stop occurs at or after it instead of the normal one (only
    takes effect in phase 1/pit_ops -- see path_analyzer.build_drive_segments'
    special_trigger_idxs). A click only registers if it lands within
    click_radius_m of an existing waypoint marker; anywhere else on the map
    is ignored. Returns the subset of `waypoints` flagged special, in their
    original order, once the figure window is closed.
    """
    fig, ax = plt.subplots(figsize=(10, 10))
    _draw_map_meters(ax, grid, resolution)
    _draw_start_circle(ax, start_circle_center_m, start_circle_radius_m)
    _draw_boulder_safety_halos(ax, boulder_centers_m, boulder_radii_m, boulder_safety_margin_m)
    _draw_zones(ax, start, resolution, landing_zone_radius_m, start_area_radius_m)

    waypoints_m = [_grid_to_m(r, c, resolution) for r, c in waypoints]
    is_special = [False] * len(waypoints)
    markers = []

    def marker_color(i):
        return "magenta" if is_special[i] else "yellow"

    for i, (x_m, y_m) in enumerate(waypoints_m):
        marker, = ax.plot(x_m, y_m, "o", color=marker_color(i), markeredgecolor="black",
                           markersize=10, zorder=5)
        label = ax.annotate(str(i + 1), (x_m, y_m), textcoords="offset points",
                             xytext=(6, 6), color=marker_color(i), fontsize=17, weight="bold", zorder=6)
        markers.append((marker, label))

    ax.set_title(title)

    # Same click-vs-drag disambiguation as pick_waypoints: a placement only
    # counts if the mouse is released within a few pixels of where it was
    # pressed, so a zoom/pan drag never toggles a marker by accident.
    CLICK_TOLERANCE_PX = 4
    press_xy = {"px": None}

    def on_press(event):
        press_xy["px"] = (event.x, event.y) if event.inaxes == ax else None

    def on_release(event):
        start_px = press_xy["px"]
        press_xy["px"] = None
        if start_px is None or event.inaxes != ax or event.xdata is None or event.ydata is None:
            return
        if (event.x - start_px[0]) ** 2 + (event.y - start_px[1]) ** 2 > CLICK_TOLERANCE_PX ** 2:
            return  # mouse moved -- a drag (zoom/pan), not a click

        if not waypoints_m:
            return
        click_m = np.array([event.xdata, event.ydata])
        dists = [float(np.linalg.norm(click_m - np.array(p))) for p in waypoints_m]
        nearest_i = int(np.argmin(dists))
        if dists[nearest_i] > click_radius_m:
            return  # click too far from any waypoint marker -- ignore

        is_special[nearest_i] = not is_special[nearest_i]
        marker, label = markers[nearest_i]
        color = marker_color(nearest_i)
        marker.set_color(color)
        label.set_color(color)
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("button_press_event", on_press)
    fig.canvas.mpl_connect("button_release_event", on_release)

    plt.show()  # blocks until the window is closed

    return [wp for wp, special in zip(waypoints, is_special) if special]


def save_route_plot(grid: np.ndarray, path: List[Tuple[int, int]], resolution: float, output_path,
                     waypoints: Optional[List[Tuple[int, int]]] = None,
                     special_waypoints: Optional[List[Tuple[int, int]]] = None,
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
                     crop_size_m: Optional[float] = None,
                     dpi: int = 300):
    """
    Save a static picture of the final planned route over the occupancy grid
    (axes in meters, resolution noted in the title), for the mission's
    output folder. If crop_center_m/crop_size_m are given, the saved image
    is cropped to a crop_size_m x crop_size_m square around crop_center_m
    instead of showing the full map -- combined with a higher dpi, this
    gives a detailed close-up without re-rasterizing anything (the source
    grid is unchanged, only the view and the output resolution change).
    special_waypoints (a subset of `waypoints`, see pick_special_waypoints)
    are drawn in magenta instead of yellow.
    """
    fig, ax = plt.subplots(figsize=(10, 10))
    _draw_map_meters(ax, grid, resolution)
    _draw_start_circle(ax, start_circle_center_m, start_circle_radius_m)
    _draw_boulder_safety_halos(ax, boulder_centers_m, boulder_radii_m, boulder_safety_margin_m)
    _draw_path_corridor(ax, path, resolution, corridor_width_m)
    _draw_zones(ax, start, resolution, landing_zone_radius_m, start_area_radius_m)

    if path:
        path_m = [_grid_to_m(r, c, resolution) for r, c in path]
        ax.plot([p[0] for p in path_m], [p[1] for p in path_m], "b-", linewidth=3, label="Path")
        ax.plot(*path_m[0], "go", markersize=12)
        ax.plot(*path_m[-1], "ro", markersize=12, label="Goal")

    if waypoints:
        special_set = set(special_waypoints) if special_waypoints else set()
        for i, (r, c) in enumerate(waypoints, start=1):
            x_m, y_m = _grid_to_m(r, c, resolution)
            color = "magenta" if (r, c) in special_set else "yellow"
            ax.plot(x_m, y_m, "o", color=color, markeredgecolor="black", markersize=10, zorder=5)
            ax.annotate(str(i), (x_m, y_m), textcoords="offset points",
                        xytext=(6, 6), color=color, fontsize=17, weight="bold", zorder=6)

    ax.set_title(f"Planned Route (resolution: {resolution:.2f} px/m)")
    ax.legend(loc="upper right")

    if crop_center_m is not None and crop_size_m is not None:
        half = crop_size_m / 2
        ax.set_xlim(crop_center_m[0] - half, crop_center_m[0] + half)
        ax.set_ylim(crop_center_m[1] - half, crop_center_m[1] + half)

    fig.savefig(output_path, dpi=dpi)
    plt.close(fig)


def save_route_plot_interactive(grid: np.ndarray, path: List[Tuple[int, int]], resolution: float, output_path,
                                 waypoints: Optional[List[Tuple[int, int]]] = None,
                                 special_waypoints: Optional[List[Tuple[int, int]]] = None,
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
                                 crop_size_m: Optional[float] = None,
                                 background_dpi: int = 400):
    """
    Interactive Plotly equivalent of save_route_plot, saved as a
    self-contained .html file (same pattern as
    conops_planner.plot_timeline_interactive). Opens in a browser with the
    same scroll-to-zoom/drag-to-pan behavior as the live matplotlib window --
    the axis re-renders at whatever zoom level you're at, unlike a static
    PNG/PDF whose ticks are baked in for the full-map view.

    The dense hazard-field layers (grid cells, boulder safety halos, path
    corridor band -- routinely hundreds of thousands of craters/boulders,
    far too many to redraw as individual interactive shapes without the
    page becoming huge/sluggish) are baked into one raster image and
    embedded as a fixed background. The actual route -- path, waypoints,
    start/landing/start-circle zones -- is redrawn as live Plotly
    traces/shapes on top, so those stay crisp and hoverable at any zoom.
    Background image detail is capped by background_dpi (pixels are spread
    over the full map by default): zooming far enough past that will start
    to blur the grid/halos/corridor even though the overlay keeps its own
    resolution -- e.g. at the default background_dpi/figsize, a 2200 m map
    only gets ~1.8 px/m, well under a metre-scale corridor band's width.

    crop_center_m/crop_size_m: like save_route_plot's same-named params --
    if given, the background raster (and the view's initial pan/zoom
    window) is restricted to a crop_size_m x crop_size_m square around
    crop_center_m instead of the full map, so the same pixel budget
    (background_dpi x figsize) lands on a much smaller area -- e.g. a 200 m
    crop gets ~20 px/m at the default settings, comfortably sharper than
    the grid's own native resolution. The image simply doesn't extend past
    the crop, so panning beyond it shows blank canvas -- this is meant as a
    detail view, not a replacement for the full-map file.
    """
    height_m = grid.shape[0] / resolution
    width_m = grid.shape[1] / resolution

    bg_fig, bg_ax = plt.subplots(figsize=(10, 10))
    bg_ax.set_position([0, 0, 1, 1])  # fill the whole figure so the raster maps 1:1 onto the view below
    _draw_map_meters(bg_ax, grid, resolution)
    _draw_boulder_safety_halos(bg_ax, boulder_centers_m, boulder_radii_m, boulder_safety_margin_m)
    _draw_path_corridor(bg_ax, path, resolution, corridor_width_m)

    if crop_center_m is not None and crop_size_m is not None:
        half = crop_size_m / 2
        img_x0, img_x1 = crop_center_m[0] - half, crop_center_m[0] + half
        img_y0, img_y1 = crop_center_m[1] - half, crop_center_m[1] + half
    else:
        img_x0, img_x1 = 0, width_m
        img_y0, img_y1 = 0, height_m
    bg_ax.set_xlim(img_x0, img_x1)
    bg_ax.set_ylim(img_y0, img_y1)
    bg_ax.axis("off")
    buf = io.BytesIO()
    bg_fig.savefig(buf, format="png", dpi=background_dpi)
    plt.close(bg_fig)
    background_uri = "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode("ascii")

    fig = go.Figure()
    fig.add_layout_image(dict(
        source=background_uri, xref="x", yref="y",
        x=img_x0, y=img_y1, xanchor="left", yanchor="top",
        sizex=img_x1 - img_x0, sizey=img_y1 - img_y0, sizing="stretch", layer="below"))

    def add_circle(center_m, radius_m, color, fill_alpha=None, dash=None):
        if center_m is None or not radius_m:
            return
        fig.add_shape(
            type="circle",
            x0=center_m[0] - radius_m, y0=center_m[1] - radius_m,
            x1=center_m[0] + radius_m, y1=center_m[1] + radius_m,
            line=dict(color=color, dash=dash, width=1.5),
            fillcolor=color if fill_alpha is not None else None,
            opacity=fill_alpha if fill_alpha is not None else 1.0)

    add_circle(start_circle_center_m, start_circle_radius_m, "cyan", dash="dash")
    if start is not None:
        start_m = _grid_to_m(start[0], start[1], resolution)
        add_circle(start_m, landing_zone_radius_m, "orange", fill_alpha=0.15)
        add_circle(start_m, start_area_radius_m, "red", fill_alpha=0.25)

    if path:
        path_m = [_grid_to_m(r, c, resolution) for r, c in path]
        fig.add_trace(go.Scatter(
            x=[p[0] for p in path_m], y=[p[1] for p in path_m],
            mode="lines", line=dict(color="blue", width=3), name="Path", hoverinfo="skip"))
        fig.add_trace(go.Scatter(
            x=[path_m[0][0]], y=[path_m[0][1]], mode="markers",
            marker=dict(color="green", size=14), name="Start", hoverinfo="skip"))
        fig.add_trace(go.Scatter(
            x=[path_m[-1][0]], y=[path_m[-1][1]], mode="markers",
            marker=dict(color="red", size=14), name="Goal", hoverinfo="skip"))

    if waypoints:
        special_set = set(special_waypoints) if special_waypoints else set()
        regular = [(i, wp) for i, wp in enumerate(waypoints, start=1) if wp not in special_set]
        special = [(i, wp) for i, wp in enumerate(waypoints, start=1) if wp in special_set]
        for group, color, name in ((regular, "yellow", "Waypoint"), (special, "magenta", "Special waypoint")):
            if not group:
                continue
            pts_m = [_grid_to_m(r, c, resolution) for _, (r, c) in group]
            fig.add_trace(go.Scatter(
                x=[p[0] for p in pts_m], y=[p[1] for p in pts_m],
                mode="markers+text", text=[str(i) for i, _ in group], textposition="top center",
                marker=dict(color=color, size=12, line=dict(color="black", width=1)), name=name))

    fig.update_xaxes(range=[img_x0, img_x1], title="X [m]", constrain="domain")
    fig.update_yaxes(range=[img_y0, img_y1], title="Y [m]", scaleanchor="x", scaleratio=1)
    fig.update_layout(
        title=f"Planned Route (resolution: {resolution:.2f} px/m)",
        height=900, width=900,
        legend=dict(x=1.02, y=1, xanchor="left"))

    fig.write_html(output_path)
