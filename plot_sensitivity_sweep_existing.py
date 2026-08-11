from src.operation import sensitivity_march_review
from user_data.config_map import baseline_config
import json


metrics = ["success_rate", "grid_occupation_mean", "tortuosity_factor_mean","amount_turns_mean"]

with open("experiments/legacy_root_outputs/experiments_n100_baseline_Dmin_crater.json") as f:
    results = json.load(f)

sensitivity_march_review.plot_sensitivity(results,parameter="Dmin_crater",metrics=metrics, fig_title="Minimal Robot Capabilities Sensitivity")
