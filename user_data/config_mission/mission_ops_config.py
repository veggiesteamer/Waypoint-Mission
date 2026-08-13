config = {
    "robot_velocity": 0.2,  #m/s

    # Per-phase stop parameters. Phase 0 = traverse (start -> first
    # waypoint), Phase 1 = pit_ops (first waypoint -> the rest) -- see
    # phase_names below. min_distance/max_distance bound the range between
    # measurement/downlink stops (averaged into the actual spacing);
    # num_pictures is how many Situational Picture tasks each ordinary stop
    # takes in that phase. special_num_pictures/special_num_spec_measurements
    # only apply to stops that land on a "special" waypoint (phase 1 only --
    # see waypoint_picker.pick_special_waypoints); they replace the normal
    # STOP_TASK_SEQUENCE with conops_planner.SPECIAL_STOP_TASK_SEQUENCE at
    # that stop, repeating picture/SPEC measurement that many times.
    "phase_params": {
        0: {"min_distance": 6, "max_distance": 10, "num_pictures": 1},
        1: {"min_distance": 1, "max_distance": 3, "num_pictures": 1,
            "special_num_pictures": 7, "special_num_spec_measurements": 7},
    },

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
    "angle_threshold_deg": 5,

    # Report metrics (mission time, distance, activity breakdown) are split
    # into two mission phases: start -> first waypoint (phase_names[0]), and
    # first waypoint -> all remaining waypoints (phase_names[1]). Rename
    # these to whatever fits the mission narrative, or set to None to skip
    # the phase breakdown and report the mission as a single whole.
    "phase_names": ("traverse", "pit_ops"),

    # Force a "localization turn" task (duration: robot_time_config.
    # localization_turn_time) every Nth measurement -- each regular drive
    # stop takes exactly one measurement, so this is every Nth stop (not
    # related to the waypoints clicked on the map, which only set the
    # route -- see conops_planner.build_mission). Set to None to disable.
    "localization_interval_measurements": 3,
}
