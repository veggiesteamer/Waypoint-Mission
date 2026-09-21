from src.globalplanner import path_planning_astar
from src.mapdata import distribution_generator
from src.operation import path_analyzer
from src.operation import conops_planner
from user_data.config_robot import robot_time_config
from user_data.config_robot import robot_power_config
from tqdm import tqdm
import matplotlib.pyplot as plt
import numpy as np

def run_simulation(config):

    map_distance_meter = config["map_size_m"]
    map_resolution = config["resolution"]
    robot_footprint = config["robot_footprint"]

    blowup = config["blowup"]
    stop_distance_range = (config["min_distance"], config["max_distance"])
    robot_velocity = config["robot_velocity"]

    # crater production
    keep_out_crater = config["keep_out_crater"]
    k_crater = config["k_crater"]
    b_crater = config["b_crater"]
    Dmin_crater = config["Dmin_crater"]
    Dmax_crater = config["Dmax_crater"]

    # rock production
    keep_out_boulder = config["keep_out_boulder"]
    k_boulder = config["k_boulder"]
    b_boulder = config["b_boulder"]
    Dmin_boulder = config["Dmin_boulder"]
    Dmax_boulder = config["Dmax_boulder"]

    # Define start and goal positions (y,x)
    start_pos = (2,2)
    goal_pos = (int(map_distance_meter*map_resolution*95/100),int(map_distance_meter*map_resolution*95/100))

    # example: generate map + path
    # Generate a crater and boulder distribution
    #start = time.perf_counter()
    center_crater, radius_crater = distribution_generator.center_radius_generator_distribution(map_distance_meter,map_resolution,keep_out_crater,robot_footprint,k_crater,b_crater,Dmin_crater,Dmax_crater)
    center_boulder, radius_boulder = distribution_generator.center_radius_generator_distribution(map_distance_meter,map_resolution,keep_out_boulder,robot_footprint,k_boulder,b_boulder,Dmin_boulder,Dmax_boulder)

    lunar_distribution_data = {
        "center": np.vstack([center_crater, center_boulder]),
        "radius": np.concatenate([radius_crater, radius_boulder]),
        "type": np.array(["crater"] * len(radius_crater) + ["boulder"] * len(radius_boulder))
    }

    #distribution_generator.distribution_vizualization(map_distance_meter, map_resolution,lunar_distribution_data,ABOVE_THREASHOLD=2)
    # Rasterizise the map
    grid = distribution_generator.rasterize_circles(lunar_distribution_data,map_distance_meter,map_resolution*map_distance_meter)
    grid_occupation = np.sum(grid) / grid.size
    # Call A* to calculate shortest path, path is a List[Tuple[int, int]]
    path = path_planning_astar.find_path_astar(grid,start_pos, goal_pos)
    #run_time_algorithm = time.perf_counter - start

    #detect failure
    success = path is not None and len(path) > 0

    # analyze path if successful
    if success:
        stats = path_analyzer.turn_analysis(path, map_resolution=map_resolution)

        segments = path_analyzer.build_drive_segments(path, map_resolution, stop_distance_range)
        timeline = conops_planner.build_mission(segments, robot_velocity, robot_time_config.config, robot_power_config.config)
        mission_time_h = max(t.end for t in timeline) / 3600 if timeline else None

        stats["mission_time_h"] = mission_time_h
        stats["amount_stops"] = len(segments)
    else:
        stats = {}

    result = {
        "amount_crater": len(center_crater),
        "amount_boulder": len(center_boulder),
        "success": success,
        "grid_occupation": grid_occupation,
        **stats
        }

    return result


def run_monte_carlo(config, n_runs):
    experiments = []
    for i in tqdm(range(n_runs)):
        record = run_simulation(config,)
        experiments.append(record)
        #print(record)
    return experiments

def aggregate_statistics(results,decimals=2):

    n_runs = len(results)

    # success handling
    successes = [r["success"] for r in results]
    n_success = sum(successes)
    success_rate = n_success / n_runs if n_runs > 0 else 0

    # collect values
    #runtimes = [r["runtime"] for r in results]
    path_lengths = [r["path_length"] for r in results if r["success"]]
    amount_crater = [r["amount_crater"] for r in results if r["success"]]
    amount_boulder = [r["amount_boulder"] for r in results if r["success"]]
    tortuosity_factor = [r["tortuosity_factor"] for r in results if r["success"]]
    amount_turns = [r["amount_turns"] for r in results if r["success"]]
    turns_per_100m = [r["turns_per_100m"] for r in results if r["success"]]
    mean_distance_before_turn = [r["mean_distance_before_turn"] for r in results if r["success"]]
    mission_time_h = [r["mission_time_h"] for r in results if r["success"]]
    grid_occupation = [r["grid_occupation"] for r in results if r["success"]]

    # compute mean/std upfront
    grid_occupation_mean = float(round(np.mean(grid_occupation), decimals)) if grid_occupation else None
    amount_crater_mean = float(round(np.mean(amount_crater), decimals)) if amount_crater else None
    amount_boulder_mean = float(round(np.mean(amount_boulder), decimals)) if amount_boulder else None

    path_length_mean = float(round(np.mean(path_lengths), decimals)) if path_lengths else None
    path_length_std  = float(round(np.std(path_lengths), decimals)) if path_lengths else None

    tortuosity_factor_mean = float(round(np.mean(tortuosity_factor), decimals)) if tortuosity_factor else None
    tortuosity_factor_std  = float(round(np.std(tortuosity_factor), decimals)) if tortuosity_factor else None

    amount_turns_mean = float(round(np.mean(amount_turns), decimals)) if amount_turns else None
    amount_turns_median = float(round(np.median(amount_turns), decimals)) if amount_turns else None
    amount_turns_std  = float(round(np.std(amount_turns), decimals)) if amount_turns else None

    turns_per_100m_mean = float(round(np.mean(turns_per_100m), decimals)) if turns_per_100m else None
    turns_per_100m_std = float(round(np.std(turns_per_100m), decimals)) if turns_per_100m else None

    mean_distance_before_turn_mean = float(round(np.mean(mean_distance_before_turn), decimals)) if mean_distance_before_turn else None

    mission_time_h_mean = float(round(np.mean(mission_time_h), decimals)) if mission_time_h else None
    mission_time_h_std = float(round(np.std(mission_time_h), decimals)) if mission_time_h else None



    summary = {
        "n_runs": n_runs,
        "n_runs_success": n_success,
        "success_rate": success_rate,
        "grid_occupation_mean": grid_occupation_mean,

        "amount_crater_mean": amount_crater_mean,
        "amount_boulder_mean": amount_boulder_mean,

        #"runtime_mean": np.mean(runtimes),
        #"runtime_std": np.std(runtimes),

        "path_length_mean": path_length_mean,
        "path_length_std": path_length_std,

        "tortuosity_factor_mean": tortuosity_factor_mean,
        "tortuosity_factor_std": tortuosity_factor_std,

        "amount_turns_mean": amount_turns_mean,
        "amount_turns_median": amount_turns_median,
        "amount_turns_std": amount_turns_std,

        "turns_per_100m_mean": turns_per_100m_mean,
        "turns_per_100m_std": turns_per_100m_std,
        "mean_distance_before_turn_mean": mean_distance_before_turn_mean,

        "mission_time_h_mean": mission_time_h_mean,
        "mission_time_h_std": mission_time_h_std
    }

    return summary

def run_experiment(config, n_runs):

    results = run_monte_carlo(config, n_runs)
    stats = aggregate_statistics(results)

    experiment_result = {
        "config": config,
        "stats": stats
    }

    return experiment_result

def run_parameter_sweep(base_config, parameter_name, values, n_runs):

    sweep_results = []

    for v in values:

        config = base_config.copy()
        config[parameter_name] = v

        result = run_experiment(config, n_runs)
        
        outcome = {
            "seed": "n/a",
            "sweep_parameter": parameter_name,
            "sweep_values": values,
            "n_runs": n_runs,
            **result
        }

        sweep_results.append(outcome)

    return sweep_results




def plot_sensitivity(results, parameter, metrics, fig_title=None):

    rows = int(np.ceil(len(metrics)/2))
    fig, axes = plt.subplots(rows, 2, figsize=(10, 4*rows))
    axes = axes.flatten()

    x = [r["config"][parameter] for r in results]

    n = len(metrics)

    #fig, axes = plt.subplots(n, 1, figsize=(6, 4*n))

    if n == 1:
        axes = [axes]  # make iterable

    for ax, metric in zip(axes, metrics):

        y = [r["stats"][metric] for r in results]

        ax.plot(x, y, marker="o")

        ax.set_title(metric)
        ax.set_xlabel(parameter)
        ax.set_ylabel(metric)

        ax.grid(True)

    if fig_title:
        fig.suptitle(fig_title)

    plt.tight_layout()  
    plt.show()