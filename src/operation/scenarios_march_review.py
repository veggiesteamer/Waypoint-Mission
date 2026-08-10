from src.globalplanner import path_planning_astar
from src.mapdata import distribution_generator
from src.operation import path_analyzer
from src.operation import conops_planner
from user_data.config_robot import robot_time_config
from user_data.config_robot import robot_power_config
import matplotlib.pyplot as plt
import numpy as np
import copy
from tqdm import tqdm



def run_simulation(config):

    map_distance_meter = config["map_size_m"]
    map_resolution = config["resolution"]
    robot_footprint = config["robot_footprint"]

    blowup = config["blowup"]
    stop_distance_m = (config["min_distance"] + config["max_distance"]) / 2
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
    goal_pos = (map_distance_meter*map_resolution*95/100,map_distance_meter*map_resolution*95/100)

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

        segments = path_analyzer.build_drive_segments(path, map_resolution, stop_distance_m)
        timeline = conops_planner.build_mission(segments, robot_velocity, robot_time_config.config, robot_power_config.config)
        mission_time_h = max(t.end for t in timeline) / 3600 if timeline else None

        stats["mission_time_h"] = mission_time_h
        stats["amount_stops"] = len(segments)
    else:
        stats = {}

    result = {
        "success": success,
        "grid_occupation":grid_occupation,
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
    grid_occupation = [r["grid_occupation"] for r in results]
    path_lengths = [r["path_length"] for r in results if r["success"]]
    tortuosity_factor = [r["tortuosity_factor"] for r in results if r["success"]]
    amount_turns = [r["amount_turns"] for r in results if r["success"]]
    turns_per_100m = [r["turns_per_100m"] for r in results if r["success"]]
    mean_distance_before_turn = [r["mean_distance_before_turn"] for r in results if r["success"]]
    mission_time_h = [r["mission_time_h"] for r in results if r["success"]]


    # compute mean/std upfront
    grid_occupation_mean = float(round(np.mean(grid_occupation), decimals)) if path_lengths else None

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
        "grid_occupation": grid_occupation_mean,

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

def merge_config(base, override):

    config = base.copy()
    config.update(override)

    return config


def extract_metric(data, metric):
    scenarios = []
    values = []

    for name, content in data.items():
        val = content["stats"].get(metric, None)
        scenarios.append(name)
        values.append(val)

    return scenarios, values


def clean_values(values):
    """Replace None with np.nan for plotting"""
    return [v if v is not None else np.nan for v in values]


# -----------------------------
# 1. SUCCESS RATE PLOT
# -----------------------------
def plot_success_rate(data):
    scenarios, values = extract_metric(data, "success_rate")
    values = clean_values(values)

    plt.figure()
    plt.bar(scenarios, values)
    plt.ylabel("Success Rate")
    plt.title("Scenario Comparison: Success Rate")
    plt.ylim(0, 1.1)

    # annotate values
    for i, v in enumerate(values):
        if not np.isnan(v):
            plt.text(i, v + 0.02, f"{v:.2f}", ha='center')

    plt.show()


# -----------------------------
# 2. PATH METRICS (GROUPED BAR)
# -----------------------------
def plot_path_metrics(data):
    metrics = [
        ("path_length_mean", "Path Length"),
        ("tortuosity_factor_mean", "Path Inefficiency"),
        ("amount_turns_mean", "Number of Turns"),
    ]

    scenarios = list(data.keys())
    x = np.arange(len(scenarios))
    width = 0.25

    plt.figure()

    for i, (metric, label) in enumerate(metrics):
        _, values = extract_metric(data, metric)
        values = clean_values(values)

        offset = (i - 1) * width
        plt.bar(x + offset, values, width, label=label)

    plt.xticks(x, scenarios)
    plt.title("Path Performance Comparison")
    plt.legend()

    plt.show()


# -----------------------------
# 3. NORMALIZED COMPARISON
# -----------------------------
def plot_relative_to_nominal(data, baseline="nominal"):
    metrics = [
        "success_rate",
        "path_length_mean",
        "tortuosity_factor_mean",
        "amount_turns_mean",
    ]

    base_stats = data[baseline]["stats"]

    scenarios = []
    rel_values = {m: [] for m in metrics}

    for name, content in data.items():
        if name == baseline:
            continue

        scenarios.append(name)
        stats = content["stats"]

        for m in metrics:
            base_val = base_stats.get(m)
            val = stats.get(m)

            if base_val in [None, 0] or val is None:
                rel_values[m].append(np.nan)
            else:
                rel_values[m].append((val - base_val) / base_val)

    x = np.arange(len(scenarios))
    width = 0.2

    plt.figure()

    for i, m in enumerate(metrics):
        offset = (i - 1.5) * width
        plt.bar(x + offset, rel_values[m], width, label=m)

    plt.xticks(x, scenarios)
    plt.axhline(0)
    plt.title("Relative Performance vs Nominal")
    plt.ylabel("Relative Change")
    plt.legend()

    plt.show()


# -----------------------------
# 4. QUICK SUMMARY TABLE (PRINT)
# -----------------------------
def print_summary_table(data):
    headers = [
        "Scenario",
        "Success",
        "PathLen",
        "Tortuosity",
        "Turns",
        "Turns/100m",
        "GridOcc",
        "MissionTime[h]"
    ]

    print("{:<15} {:<10} {:<10} {:<12} {:<8} {:<12} {:<10} {:<14}".format(*headers))

    for name, content in data.items():
        s = content["stats"]

        row = [
            name,
            s.get("success_rate"),
            s.get("path_length_mean"),
            s.get("tortuosity_factor_mean"),
            s.get("amount_turns_mean"),
            s.get("turns_per_100m_mean"),
            s.get("grid_occupation"),
            s.get("mission_time_h_mean"),
        ]

        def fmt(x):
            return "-" if x is None else f"{x:.2f}"

        print("{:<15} {:<10} {:<10} {:<12} {:<8} {:<12} {:<10} {:<14}".format(
            name,
            fmt(row[1]),
            fmt(row[2]),
            fmt(row[3]),
            fmt(row[4]),
            fmt(row[5]),
            fmt(row[6]),
            fmt(row[7]),
        ))

def flatten_data(data):
    rows = []

    for name, content in data.items():
        row = {"scenario": name}

        # config parameters
        for k, v in content["config"].items():
            row[k] = v

        # stats
        for k, v in content["stats"].items():
            row[k] = v

        rows.append(row)

    return rows

