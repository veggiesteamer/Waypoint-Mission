import json
from pathlib import Path

from src.operation import marius_hills_mission
from user_data.config_mission import mission_ops_config
from user_data.config_robot import robot_power_config
from user_data.config_robot import robot_time_config

OUTPUT_DIR = Path("experiments/marius_hills")

with open(OUTPUT_DIR / "marius_hills_experiment.json") as f:
    experiment = json.load(f)

mission = marius_hills_mission.run_mission_planning(
    experiment, mission_ops_config.config, robot_time_config.config, robot_power_config.config,
    timeline_image_path=OUTPUT_DIR / "Marius_Hills_Mission_Timeline.png")

with open(OUTPUT_DIR / "marius_hills_mission.json", "w") as f:
    json.dump(mission, f, indent=2)
    f.write("\n")
