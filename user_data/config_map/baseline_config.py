config = {
    "num_runs": 5,
    "map_size_m": 100,
    "resolution": 5,
    "robot_footprint": 0,
    "num_craters": 50,

    "blowup": 1.1,
    "min_distance": 6,   #m, lower bound of the range between measurement/downlink stops
    "max_distance": 10,  #m, upper bound of the range between measurement/downlink stops

    "robot_velocity": 0.2,  #m/s

    "detection_threashold": 1,#meter

    # crater production function
    "keep_out_crater": 0,
    "k_crater": 0.0537,
    "b_crater": 2.02,
    "Dmin_crater": 0.5,
    "Dmax_crater": 50,

    # rock production function
    "keep_out_boulder": 0.,
    "k_boulder": 0.00126,
    "b_boulder": 2.627,
    "Dmin_boulder": 0.2,
    "Dmax_boulder": 10
}
