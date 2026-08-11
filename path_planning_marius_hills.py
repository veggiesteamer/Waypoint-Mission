import json
import time
from pathlib import Path

from src.operation import marius_hills_review
from user_data.config_map import marius_hills_config

OUTPUT_DIR = Path("experiments/marius_hills")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

start = time.perf_counter()

experiment = marius_hills_review.run_simulation(
    marius_hills_config.config,
    dist_image_path=OUTPUT_DIR / "Marius_Hills_Mapped.png",
    raster_image_path=OUTPUT_DIR / "Marius_Hills_Rasterized.png",
    path_image_path=OUTPUT_DIR / "Marius_Hills_Path.png",
)

print(f"Runtime: {time.perf_counter() - start:.2f}s")
print(experiment["result"])

with open(OUTPUT_DIR / "marius_hills_experiment.json", "w") as f:
    json.dump(experiment, f, indent=2)
    f.write("\n")
