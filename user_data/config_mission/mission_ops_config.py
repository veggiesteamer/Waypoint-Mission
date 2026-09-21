config = {
    "robot_velocity": 0.2,  #m/s

    # Per-phase stop parameters. Phase 0 = traverse (start -> first
    # waypoint), Phase 1 = pit_ops (first waypoint -> the rest) -- see
    # phase_names below. min_distance/max_distance bound the range between
    # measurement/downlink stops -- each "stretch" of path within this phase
    # gets split into equal-length legs that best fit its own total distance
    # inside this range (see path_analyzer.distribute_stretch). A stretch
    # isn't the whole phase: it also breaks early at a real corner in the
    # route (stop_angle_threshold_deg below decides what counts as one --
    # deliberately coarser than angle_threshold_deg, which is far too
    # sensitive to staircase/simplification noise for this) once
    # turn_stop_floor_fraction * min_distance has accumulated since the
    # last stop -- so a long, mostly-straight phase doesn't collapse into
    # one repeated number, AND a real corner is always a stop even if
    # that means a leg shorter than min_distance -- or after
    # max_stretch_multiple * max_distance with no qualifying corner at all
    # (see path_analyzer.build_drive_segments' docstring). num_pictures is
    # how many Situational Picture tasks each ordinary stop takes in that
    # phase.
    #
    # special_sequence only applies to stops that land on a "special"
    # waypoint (phase 1 only -- see waypoint_picker.pick_special_waypoints):
    # an ordered list of conops_planner.TASK_CATALOG_FOR_SPECIAL_SEQUENCE
    # keys that completely replaces the normal STOP_TASK_SEQUENCE at that
    # stop -- both ordinary STOP_TASK_SEQUENCE keys (e.g. "spec_measurement")
    # and SPECIAL_ONLY_TASK_CATALOG keys (currently just "turn", a
    # re-pointing maneuver, never usable at an ordinary stop) are valid.
    # List a key multiple times to repeat it (order/repeats/omissions are
    # all free -- this is the whole special-stop activity plan,
    # self-contained). Must contain "image_downlink" exactly once.
    #
    # task_intervals controls how often each measurement/downlink actually
    # runs, at *ordinary* (non-special) stops in that phase -- e.g. 3 means
    # "only every 3rd stop" (a stop-count cadence, unlike
    # localization_interval_distance_m below, which is distance-based).
    # Keys not listed (or set to 1) run every stop, matching today's
    # behavior.
    # picture/nap/image_downlink aren't controlled this way: image_downlink
    # always fires every stop (it anchors earth_analysis's start time),
    # picture has its own num_pictures above, and nap always runs.
    #
    "phase_params": {
        0: {"min_distance": 1, "max_distance": 10, "num_pictures": 1,
            "task_intervals": {
                "grav_measurement": 1,
                "spec_measurement": 10,
                "scicam_measurement": 2,
                "relay": 1,
            }},
        1: {"min_distance": 0.5, "max_distance": 2.5, "num_pictures": 1,
            "task_intervals": {
                "grav_measurement": 3,
                "spec_measurement": 10,
                "scicam_measurement": 5,
                "relay": 1,
            },
            # Display name for special stops in the mission report/summary
            # (see marius_hills_mission.generate_mission_report) -- shown as
            # its own "Nx" line, separate from the individual task rows it's
            # made of.
            "special_sequence_name": "Pit Wall Inspection",
            # spec_warmup runs once, right before the first spec_measurement --
            # the instrument is assumed to stay warm for the rest of the
            # inspection rather than needing to re-warm before each batch
            # below (see conops_planner.STOP_TASK_SEQUENCE for the same
            # warmup+measurement pairing at ordinary stops, where it does
            # repeat every time). The {"batch": ...} entry runs "batch"
            # (spec_measurement) batch_size at a time, with one
            # scicam_measurement between consecutive batches (not after the
            # last one), until `total` spectra have been taken -- expanded
            # by conops_planner.generate_waypoint_tasks via
            # conops_planner.batched_sequence. Change batch_size/total here
            # freely; nothing else needs updating.
            "special_sequence": [
                "picture", "image_downlink", "scicam_measurement", "spec_warmup", "scicam_measurement",
                {"batch": "spec_measurement", "batch_size": 5, "total": 61,
                 "interleave": ["scicam_measurement"]},
                "grav_measurement", "relay", "nap",
            ]},
    },

    # Sub-phase regions: a window (by distance walked along the path, not
    # stop count) around a waypoint marked on the map -- same click pass as
    # special waypoints, see waypoint_picker.pick_special_waypoints/
    # SPECIAL_WAYPOINT_CATEGORIES -- where stop spacing AND task cadence/
    # content are entirely swapped to this dict instead of the parent
    # phase's phase_params. Same shape as one phase_params[phase] dict
    # (min_distance/max_distance here directly change how often the rover
    # stops in this stretch too, not just which measurements fire), plus
    # its own half_width_m. Keyed by the same waypoint-category key the
    # picker uses -- currently just "grav_dense_center" -- see
    # path_analyzer.build_drive_segments' subphase_windows and
    # conops_planner.build_mission's subphase_params. A waypoint marked
    # into a category with no entry here is effectively a no-op (falls
    # back to an empty params dict).
    "subphase_params": {
        "grav_dense_center": {
            "half_width_m": 25,
            "min_distance": 3, "max_distance": 5, "num_pictures": 1,
            "task_intervals": {
                "grav_measurement": 1,
                "spec_measurement": 20,
                "scicam_measurement": 4,
                "relay": 2,
            },
        },
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
    # to fall back to path_analyzer's own default (10 deg). This is *not*
    # what decides where a phase_params stretch ends early at a corner (see
    # above) -- that's stop_angle_threshold_deg below, deliberately
    # separate and coarser, since this value is tuned for realistic
    # turn-time costing and is far too sensitive (flags a "turn" every
    # ~1-2m on a real route) to also gate stop placement.
    "angle_threshold_deg": 5,

    # How many max_distance's worth of distance a corner-free stretch (see
    # phase_params above) can run before it's forced to split anyway.
    # Uncomment/set to override path_analyzer.build_drive_segments' own
    # default (10).
    # "max_stretch_multiple": 10,

    # Min heading deviation (deg) that counts as a real corner for stretch-
    # closing purposes (see phase_params above) -- coarser than
    # angle_threshold_deg on purpose, so small-angle staircase/simplification
    # wobble doesn't force a stop, but a genuinely sharp bend in the route
    # does. Uncomment/set to override path_analyzer.build_drive_segments'
    # own default (30 deg).
    # "stop_angle_threshold_deg": 30,

    # Fraction of a phase/subphase's own min_distance that must have
    # accumulated since the last stop before a detected corner (see
    # stop_angle_threshold_deg above) is allowed to close a stretch there.
    # 1.0 would guarantee every stretch is a feasible distribute_stretch
    # total (never undershoots min_distance) but can walk straight through
    # a corner that's close to the last stop; lower values let closely-
    # spaced real corners each get their own stop, at the cost of some legs
    # coming out shorter than min_distance. Uncomment/set to override
    # path_analyzer.build_drive_segments' own default (0.3).
    # "turn_stop_floor_fraction": 0.3,

    # Report metrics (mission time, distance, activity breakdown) are split
    # into two mission phases: start -> first waypoint (phase_names[0]), and
    # first waypoint -> all remaining waypoints (phase_names[1]). Rename
    # these to whatever fits the mission narrative, or set to None to skip
    # the phase breakdown and report the mission as a single whole.
    "phase_names": ("traverse", "pit_ops"),

    # Whether the standing/settle task (robot_time_config.standing_time,
    # "Standing in any pose (braking)") is inserted after driving/turning
    # and before any image -- see conops_planner.generate_waypoint_tasks.
    # Set to False to disable it mission-wide without touching the
    # Excel-derived time/power values.
    "standing_enabled": True,

    # Force a localization sequence roughly every N meters walked --
    # cumulative distance across the whole route, independent of local stop
    # spacing (not related to the waypoints clicked on the map, which only
    # set the route -- see conops_planner.build_mission). Tracked as one
    # running total across phases/subphases, so it stays ~30m apart even
    # where stops are much closer together (e.g. phase 1's 1-3m spacing or
    # the grav_dense_center subphase's 4-6m) -- fires on whichever stop's
    # drive leg crosses the threshold, then resets. Set to None to disable.
    "localization_interval_distance_m": 30,
    # What runs each time the above fires -- an ordered list of
    # conops_planner.TASK_CATALOG_FOR_SPECIAL_SEQUENCE keys, same vocabulary
    # as special_sequence above. "turn" entries reuse robot_time_config.
    # turn_time (the "Spot Turn" value, same as the automatic per-leg turn)
    # but display as "localization turn"; every other key uses its own
    # normal display name/duration/power. Here: two re-pointing turns, then
    # a SciCAM science image. None (default) falls back to a single turn.
    "localization_sequence": ["turn", "turn", "scicam_measurement",
                              "turn", "turn", "scicam_measurement",
                              "turn", "turn", "scicam_measurement",
                              "turn", "turn", "scicam_measurement",
                              "turn", "turn", "scicam_measurement",
                              "turn", "turn", "scicam_measurement"],

    # Two independent one-time blocks at the very start of the timeline
    # (t=0, before any drive segment): rover egress from the lander, then
    # initial system commissioning. Each costs its own fixed amount of
    # time/energy but isn't a drive/measurement activity, so neither is
    # part of any phase's stop sequence (see conops_planner.build_mission).
    # duration_s/energy_wh set each block directly -- power_w is derived,
    # same convention as locomotion_experiment_config.py. Placeholder
    # values below -- tune to the real egress/commissioning timeline. Set
    # either to None to disable it independently of the other.
    "egress": {"duration_s": 4 * 3600, "energy_wh": 8},
    "commissioning": {"duration_s": 12 * 3600, "energy_wh": 12},

    # Nap duration: "curve" follows a lunar-day thermal/power cycle (see
    # nap_curve below and conops_planner.nap_duration_s); "constant" uses
    # nap_constant_min directly, in minutes -- independent of
    # robot_time_config.nap_time (which is Excel-derived and only used as
    # nap_curve's target average, or as the flat fallback if nap_mode is
    # neither of these). Real synodic lunar day (sunrise to sunrise); local
    # solar noon sits at a quarter of the way through it. mission_start_
    # lunar_hour anchors to *landing* (t=0, the start of egress above, if
    # set) -- 0 = landing right at sunrise -- not to when ops/traverse
    # actually begins, which is mission_start_lunar_hour + egress's and
    # commissioning's combined duration (whichever of those are set).
    "nap_mode": "constant",  # "curve" or "constant"
    "nap_constant_min": 10,
    "lunar_day_hours": 708.7,
    "mission_start_lunar_hour": 12,
    "nap_curve": {
        # longer naps while recharging shortly after sunrise
        "morning_peak_hour": 24,
        "morning_peak_amplitude_s": 30*60,
        "morning_peak_width_h": 15,
        # longer naps for extra cooldown around local solar noon
        # (708.7 / 4 = 177.2)
        "noon_peak_hour": 177.2,
        "noon_peak_amplitude_s": 30*60,
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
