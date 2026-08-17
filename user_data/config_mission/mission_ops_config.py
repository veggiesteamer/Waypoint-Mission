config = {
    "robot_velocity": 0.2,  #m/s

    # Per-phase stop parameters. Phase 0 = traverse (start -> first
    # waypoint), Phase 1 = pit_ops (first waypoint -> the rest) -- see
    # phase_names below. min_distance/max_distance bound the range between
    # measurement/downlink stops (averaged into the actual spacing);
    # num_pictures is how many Situational Picture tasks each ordinary stop
    # takes in that phase.
    #
    # special_sequence only applies to stops that land on a "special"
    # waypoint (phase 1 only -- see waypoint_picker.pick_special_waypoints):
    # an ordered list of conops_planner.TASK_CATALOG keys that completely
    # replaces the normal STOP_TASK_SEQUENCE at that stop. List a key
    # multiple times to repeat it (order/repeats/omissions are all free --
    # this is the whole special-stop activity plan, self-contained). Must
    # contain "image_downlink" exactly once.
    #
    # task_intervals controls how often each measurement/downlink actually
    # runs, at *ordinary* (non-special) stops in that phase -- e.g. 3 means
    # "only every 3rd stop", same "every Nth" idea as
    # localization_interval_measurements below, just per task. Keys not
    # listed (or set to 1) run every stop, matching today's behavior.
    # picture/nap/image_downlink aren't controlled this way: image_downlink
    # always fires every stop (it anchors earth_analysis's start time),
    # picture has its own num_pictures above, and nap always runs.
    "phase_params": {
        0: {"min_distance": 8, "max_distance": 12, "num_pictures": 1,
            "task_intervals": {
                "grav_measurement": 1,
                "spec_measurement": 0.1,
                "scicam_measurement": 0.5,
                "relay": 1,
            }},
        1: {"min_distance": 1, "max_distance": 3, "num_pictures": 1,
            "task_intervals": {
                "grav_measurement": 0.2,
                "spec_measurement": 0.01,
                "scicam_measurement": 0.2,
                "relay": 0.2,
            },
            # Display name for special stops in the mission report/summary
            # (see marius_hills_mission.generate_mission_report) -- shown as
            # its own "Nx" line, separate from the individual task rows it's
            # made of.
            "special_sequence_name": "Pit Wall Inspection",
            "special_sequence": [
                "picture", "image_downlink", "scicam_measurement", "spec_measurement", "scicam_measurement", "spec_measurement", "scicam_measurement", "spec_measurement",
                "scicam_measurement", "spec_measurement", "scicam_measurement", "spec_measurement", "scicam_measurement", "spec_measurement", "scicam_measurement", "spec_measurement",
                "scicam_measurement", "spec_measurement", "scicam_measurement", "spec_measurement", "scicam_measurement", "spec_measurement", "grav_measurement",  "relay", "nap",
            ]},
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

    # Nap duration follows a lunar-day thermal/power cycle instead of the
    # flat robot_time_config.nap_time -- see conops_planner.nap_duration_s.
    # Real synodic lunar day (sunrise to sunrise); local solar noon sits at
    # a quarter of the way through it. mission_start_lunar_hour is where
    # your mission begins on that clock -- 0 = starting right at sunrise.
    # Set nap_curve to None to go back to the flat nap_time.
    "lunar_day_hours": 708.7,
    "mission_start_lunar_hour": 0,
    "nap_curve": {
        # longer naps while recharging shortly after sunrise
        "morning_peak_hour": 24,
        "morning_peak_amplitude_s": 300,
        "morning_peak_width_h": 15,
        # longer naps for extra cooldown around local solar noon
        # (708.7 / 4 = 177.2)
        "noon_peak_hour": 177.2,
        "noon_peak_amplitude_s": 300,
        "noon_peak_width_h": 20,
    },

    # Standalone locomotion-experiment block(s) -- see conops_planner.
    # LOCOMOTION_TASK_CATALOG/generate_locomotion_experiment_tasks and
    # user_data/config_robot/locomotion_experiment_config.py for the
    # tasks' own time/power values. trigger_after_special_occurrence lists
    # which special-waypoint stop(s), by order (1 = the first one hit in
    # the mission, 2 = the second, ...), get a block inserted right after
    # them; sequence is the ordered list of locomotion_experiment_config
    # keys run at each of those points (repeat a key to run it more than
    # once). Set to None to disable.
    "locomotion_experiment": {
        "trigger_after_special_occurrence": [1, 2],
        "sequence": ["locomotion_test"],
    },
}
