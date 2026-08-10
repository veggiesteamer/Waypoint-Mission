import numpy as np
import pandas as pd
import matplotlib as plt
from src.mapdata import distribution_generator
from src.globalplanner import path_planning_astar
import time
import json


start = time.perf_counter()
# Load data
df = pd.read_csv('user_data/map_data/Marius_Hills_Pit/classified_craters.csv')
# rename columns: x = lon, y = lat
df.rename(columns={'x_coord':'lon', 'y_coord': 'lat'}, inplace=True)
df.head()

lat0 = df['lat'].mean()
lon0 = df['lon'].mean()

R_moon = 1737400

df['x'] = np.radians(df['lon'] - lon0) * R_moon * np.cos(np.radians(lat0))
df['y'] = np.radians(df['lat'] - lat0) * R_moon


# assume x, y are numpy arrays
#df['x_norm'] = (df['x'] + 1000) / 2000
#df['y_norm'] = (df['y'] + 1000) / 2000
df['x_norm'] = (df['x'] - df['x'].min()) / (df['x'].max() - df['x'].min())
df['y_norm'] = (df['y'] - df['y'].min()) / (df['y'].max() - df['y'].min())

map_distance_meter = 2200 #df['x'].max() - df['x'].min()
map_resolution = 5


center_MHP = np.column_stack((df['x_norm'],df['y_norm']))
radius_MHP = np.round(df['Diam_km'] * 1000 /2, 4)
min_radius_MHP = radius_MHP.min()
print(min_radius_MHP)
ABOVE_THREASHOLD = min_radius_MHP

robot_footprint = 0.0
keep_out_crater = 0#percent of Diameter
k_crater = 0.0084 #0.0537 #
b_crater = 2.02
Dmin_crater = 0.5
Dmax_crater = 20

# Rock production function

keep_out_boulder = 0 #percent
k_boulder = 0.00126
b_boulder = 2.627
Dmin_boulder = 0.2
Dmax_boulder = 10

# Define start and goal positions (y,x)
start_pos = (2,2)
goal_pos = (map_distance_meter*map_resolution*50/100,map_distance_meter*map_resolution*50/100)

inst = time.perf_counter()
print(inst-start)

center_crater, radius_crater = distribution_generator.center_radius_generator_distribution(map_distance_meter,map_resolution,keep_out_crater,robot_footprint,k_crater,b_crater,Dmin_crater,Dmax_crater)
print(radius_crater.max())
center_boulder, radius_boulder = distribution_generator.center_radius_generator_distribution(map_distance_meter,map_resolution,keep_out_boulder,robot_footprint,k_boulder,b_boulder,Dmin_boulder,Dmax_boulder)

lunar_distribution_data = {
    "center": np.vstack([center_MHP, center_crater, center_boulder]),
    "radius": np.concatenate([radius_MHP, radius_crater, radius_boulder]),
    "type": np.array(["MHP_crater"] * len(radius_MHP) + ["crater"] * len(radius_crater) + ["boulder"] * len(radius_boulder)
    )
}


inst = time.perf_counter()
print("Distribution Time: ",inst-start)

#ax = distribution_generator.distribution_vizualization(map_distance_meter,map_resolution,lunar_distribution_data, ABOVE_THREASHOLD)
ax = distribution_generator.distribution_vizualization_layers(map_distance_meter, map_resolution, lunar_distribution_data,show_MHP=True, show_craters=True, show_boulders=True)
ax.figure.savefig(f"Marius_Hills_Mapped.png", dpi=300)

inst = time.perf_counter()
print("Distribution picture: ", inst-start)

grid = distribution_generator.rasterize_circles(lunar_distribution_data,map_distance_meter,map_resolution*map_distance_meter)
ax = distribution_generator.map_vizalization(grid,map_resolution,map_distance_meter)
grid_occupation = np.sum(grid) / grid.size
ax.figure.savefig(f"Marius_Hills_Rasterized.png", dpi=300)

inst = time.perf_counter()
print("Time grid ",inst-start)
print("Grid occupation ",grid_occupation)

# Call A* to calculate shortest path, path is a List[Tuple[int, int]]
    
path = path_planning_astar.find_path_astar(grid,start_pos, goal_pos)
path_planning_astar.visualize_path(grid,path)

inst = time.perf_counter()
print("time path ", inst-start)
with open("experiments.json", "w") as f:
    json.dump(path, f)
    f.write("\n")