from src.globalplanner import path_planning_astar
from src.mapdata import distribution_generator
from src.operation import path_analyzer
from src.operation import conops_planner
from user_data.config_robot import robot_power_config
from user_data.config_robot import robot_time_config
import matplotlib.pyplot as plt
import numpy as np
import time
import json
from pathlib import Path


# Run A* Algorithm

map_distance_meter = 100
map_resolution = 5 # pixels per meter
robot_footprint = 0 #radius in meter
blowup = 1.1 # to make sure that small craters are captured


# Cater production function
#seed_value = 42
keep_out_crater = 0#percent of Diameter
k_crater =  0.0537 #0.0084 #
b_crater = 2.02
Dmin_crater = 0.5
Dmax_crater = 100

# Rock production function

keep_out_boulder = 0 #percent
k_boulder =  0.00126
b_boulder = 2.6278
Dmin_boulder = 0.2
Dmax_boulder = 10

ABOVE_DETECTION_THREASHOLD = 2

# Define start and goal positions (y,x)
start_pos = (2,2)
goal_pos = (map_distance_meter*map_resolution*95/100,map_distance_meter*map_resolution*95/100)

# Generate a crater and boulder distribution
#it = range(1)
#for i in it:
center_crater, radius_crater = distribution_generator.center_radius_generator_distribution(map_distance_meter,map_resolution,keep_out_crater,robot_footprint,k_crater,b_crater,Dmin_crater,Dmax_crater)
center_boulder, radius_boulder = distribution_generator.center_radius_generator_distribution(map_distance_meter,map_resolution,keep_out_boulder,robot_footprint,k_boulder,b_boulder,Dmin_boulder,Dmax_boulder)
#print(len(center_crater))

lunar_distribution_data = {
    "center": np.vstack([center_crater, center_boulder]),
    "radius": np.concatenate([radius_crater, radius_boulder]),
    "type": np.array(["crater"] * len(radius_crater) + ["boulder"] * len(radius_boulder)
    )
}

centers = np.vstack([center_crater, center_boulder])
radii = np.concatenate([radius_crater,radius_boulder])


resolutions = [5]#range(1)
#ax = distribution_generator.distribution_vizualization(map_distance_meter, map_resolution,lunar_distribution_data,ABOVE_DETECTION_THREASHOLD)
ax = distribution_generator.distribution_vizualization_layers(map_distance_meter, map_resolution, lunar_distribution_data, show_MHP=False, show_craters=True, show_boulders=True)

ax.figure.savefig(f"dist_{map_resolution}.png", dpi=300)
plt.close(ax.figure)

for map_resolution in resolutions:

# Rasterizise the map
    grid = distribution_generator.rasterize_circles(lunar_distribution_data,map_distance_meter,map_resolution*map_distance_meter)
    ratio = np.sum(grid) / grid.size
#grid = distribution_generator.mask_generator(map_distance_meter,map_resolution,centers, radii,blowup)
    ax = distribution_generator.map_vizalization(grid,map_resolution,map_distance_meter)
    ax.figure.savefig(f"map_{map_resolution}.png", dpi=300)
    plt.close(ax.figure)
# Call A* to calculate shortest path, path is a List[Tuple[int, int]]
    path = path_planning_astar.find_path_astar(grid,start_pos, goal_pos)
    path_planning_astar.visualize_path(grid,path)


#path_analyzer.analyze_path(path)
waypoint,distances = path_analyzer.compute_waypoints3(path,step=10)
tortuosity = path_analyzer.calculate_tortuosity(waypoint,distances)
#path_planning_astar.visualize_path_and_distribution(grid,path,map_distance_meter,lunar_distribution_data,waypoint)

# Mission timeline: stop every stop_distance_m (midpoint of the range) to take
# a measurement and downlink data/picture; count turns per drive leg in between.
stop_distance_min = 6 #m
stop_distance_max = 10 #m
stop_distance_m = (stop_distance_min + stop_distance_max) / 2

turn_stats = path_analyzer.turn_analysis(path, map_resolution=map_resolution)
segments = path_analyzer.build_drive_segments(path, map_resolution, stop_distance_m)


'''
for i in:
    do the Run
    take the parameters
    create a new Name
    save document
    to save: path, waypoints, distances, tortuosity?
''' 

# Mission definition

robot_velocity = 0.2


def decision_time_calculator(waypoint, distance, robot_velocity,config):
    const = 700/10
    total_time = 168*3600
    turtuosity, distance_traverse, distance_euclidean = path_analyzer.calculate_tortuosity(waypoint, distances)
    communication_mission = np.size(waypoint) * (config["picture_time"] + config["relay_time"] + config["uplink_time"])
    walking_mission_time = distance_traverse / robot_velocity
    science_mission_time = const * config["measurement_time"]
    mission_time = communication_mission + walking_mission_time + science_mission_time
    decision_time = (total_time - mission_time) / np.size(waypoint) / 3600
    return mission_time, decision_time

from collections import defaultdict

def generate_mission_report(tasks,waypoint,distances,config):
    tortuosity, distance_traverse, distance_euclidean = path_analyzer.calculate_tortuosity(waypoint, distances)
    communication_mission_time = np.size(waypoint) * (config["picture_time"] + config["relay_time"] + config["uplink_time"])
    walking_mission_time = distance_traverse / robot_velocity
    science_mission_time = np.size(waypoint) * config["measurement_time"]
    earth_decision_time = np.size(waypoint) * (config["earth_analysis_time"])
    mission_time = communication_mission_time + walking_mission_time + science_mission_time

    durations = defaultdict(float)

    for task in tasks:
        durations[task.name] += task.end - task.start

    mission_start = min(t.start for t in tasks)
    mission_end = max(t.end for t in tasks)
    #mission_time = mission_end - mission_start


    waypoint_count = np.size(waypoint)

    def report_line(name, value):
        print(f"{name:25s}: {value}")

    print("\n============== Mission Summary ==============\n")

    report_line("Total mission time", f"{mission_time/3600:.2f} h")
    report_line("Walking distance", f"{distance_traverse:.2f} m")
    report_line("Point to point distance", f"{distance_euclidean:.2f} m")
    report_line("Tortuosity factor", f"{tortuosity:.2f} ")
    report_line("Number of waypoints", f"{waypoint_count} \n")

    print("Activity breakdown")
    print("---------------------------------------------")

    report_line("walking", f"{walking_mission_time/3600:.2f} h")
    report_line("communication", f"{communication_mission_time/3600:.2f} h")
    report_line("science", f"{science_mission_time/3600:.2f} h")
    #report_line("earth decision", f"{earth_decision_time/3600:.2f} h")

    print("\nUtilization")
    print("---------------------------------------------")

    #print(f"Rover activity: {100*rover_time/mission_time:.1f} %")
    #print(f"Earth processing: {100*earth_time/mission_time:.1f} %")

    if waypoint_count > 0:
        print(f"\nAverage time per waypoint: {mission_time/waypoint_count/60:.2f} min")

    print("\n=============================================\n")


timeline = conops_planner.build_mission(segments,robot_velocity,robot_time_config.config,robot_power_config.config)
conops_planner.plot_timeline(timeline)
mission_time, decision_time = decision_time_calculator(waypoint,distances,robot_velocity,robot_time_config.config)

#generate_mission_report(timeline,waypoint,distances,conops_planner.config)



"""
ToDo: add proper labels
add a distance between the tasks
add a break to seperate
"""

#data = {"path": [path]}
#with open("data.json", "w") as f:
#    json.dump(data,f)