config = {
    "csv_path": "user_data/map_data/Marius_Hills_Pit/classified_craters.csv",
    "pit_outline_path": "user_data/map_data/Marius_Hills_Pit/MHP_Vertices.txt",

    "map_size_m": 2200,
    "resolution": 5, #px/m -- keep >= 1/Dmin_boulder or small objects over-block their grid cell
    "robot_footprint": 0.,  # m -- used to dilate obstacles in the grid map, and to compute the robot corridor width
    "goal_fraction": 0.5,  # goal placed at this fraction of the map extent, not the far corner
    "planning_downsample_factor": 1,  # only needed for the slow pure-Python "astar" planner; pyastar2d doesn't need it and it can falsely block all paths at higher densities (max-pooling is obstacle-conservative)
    "planner": "pyastar",  # "pyastar" (fast, compiled) or "astar" (pure-Python, has the progress bar)
    "simplify_path": True,  # collapse the A* grid-staircase into straight line-of-sight segments (see path_analyzer.simplify_path_line_of_sight) -- every kept segment is re-checked against the grid, so it can't cut through an obstacle

    # visual context only (not enforced during planning) -- placeholder sizes,
    # both centered on the rover start position
    "landing_zone_radius_m": 100,  # expected lander touchdown dispersion radius
    "start_area_radius_m": 10,    # rover starting-position uncertainty radius

    # more visual-only safety-margin context: a faint gray halo around each
    # boulder, and a faint band the width of the rover along the planned path
    "boulder_safety_margin_m": 5,
    "robot_corridor_width_m": 6,

    # map-plot only: craters/boulders with radius below this are drawn faint
    # (pastel color, lower alpha) to indicate they're near/below map resolution
    "faint_radius_threshold_m": 2,

    # rover start point: fixed at a given bearing on a circle around a center
    # point, instead of clicked freely -- angle_deg is measured counterclockwise
    # from +X (standard math convention: 0 deg = east of center, 90 deg = north)
    "start_circle_center_m": (1100, 1100),
    "start_circle_radius_m": 500,
    "start_angle_deg": 220,

    # high-resolution cropped route picture, saved alongside the full map view
    "crop_center_m": (1100, 1100),
    "crop_size_m": 500,
    "crop_dpi": 600,

    # crater production function (synthetic layer, below MHP detection threshold)
    "keep_out_crater": 1.5,
    "k_crater": 0.0084,
    "b_crater": 2.02,
    "Dmin_crater": 0.5,
    "Dmax_crater": 20,

    # rock production function
    "keep_out_boulder": 1.5,
    "k_boulder": 0.00126,
    "b_boulder": 2.627,
    "Dmin_boulder": 0.2,
    "Dmax_boulder": 10
}
