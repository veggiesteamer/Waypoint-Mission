config = {
    "robot_velocity": 0.2,  #m/s

    "min_distance": 6,   #m, lower bound of the range between measurement/downlink stops
    "max_distance": 10,  #m, upper bound of the range between measurement/downlink stops

    # Baseline distance (m) used to detect a real turn in the planned path (see
    # path_analyzer.build_drive_segments/turn_analysis): heading is compared
    # window_m/2 before vs. after each point, walked along the path itself, so
    # single-pixel grid-staircase moves aren't miscounted as turns. Omit this
    # key to fall back to path_analyzer's own default (1.0 m).
    "window_m": 1.0,

    # Max heading deviation (deg) the rover can drive through continuously
    # (over window_m) without stopping to pivot -- above this, the leg is
    # counted as a turn costing turn_time (robot_time_config). Omit this key
    # to fall back to path_analyzer's own default (10 deg).
    "angle_threshold_deg": 30,
}
